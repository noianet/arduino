/***********************************************************************
    Board: Arduino MKR 1500 with Ubox SARA R4
    AT commands: https://www.u-blox.com/sites/default/files/SARA-R4_ATCommands_(UBX-17003787).pdf
    app dev guide: https://www.u-blox.com/en/docs/UBX-18019856
    hw datasheet: https://www.u-blox.com/sites/default/files/SARA-R4_DataSheet_(UBX-16024152).pdf

     Modify: ~/.arduino15/packages/arduino/hardware/samd/1.8.11/boards.txt:: (added low power, removed power chip control (using external charger)
     mkrnb1500.build.extra_flags=-DUSE_ARDUINO_MKR_PIN_LAYOUT -D__SAMD21G18A__ {build.usb_flags} -DVERY_LOW_POWER

    Modified Mysensors serial gateway for raw udp over mobile NBIiT/LTE-M.  Buffer stack to store incoming data

    Note: Only reads mysensors float values. Incoming NRF packets are stored via interrupt in the message stack awaiting transmit loop pickup.
    Reads internal temp sensor immediately after wakeup and adds reading to buffer - OK/consistent and usable as a thermometer.
    Listens for 11 minutes, then sleeps 49 (NRF sensor nodes runs at a 10min transmit interval)

    Received NRF packets are stored in a FIFO buffer while awake. Then wakes mobile and sends all just before sleep routine .
    Sends a xx/14 "1" header to inticate start FIFO buffer, and ends with relay statistics and a xx/14 "0" to indicate end transmission.
    Modem sleeps as much as possible, only wakes right before sleep, burst send from FIFO, and then put to sleep before microcontroller 49min sleep routine.

    LED flashes on NRF packet receive while awake, and constant light while modem conect. In 49min sleep routine flashes every 8s to indicate still alive.

    Hardware watchdog routine  resets micro  if code freeze (incl. should caktch some while() connecting loops if faulty modem).

    NBNB! Always run with a connected battery to avoid brownout crashes..And seems important to connect battery first, then USB (some stupid battery protection in HW?).
    Needs external battery charger since onboard limited to <0.1A and 4hr max chargetime....Seems like MKR 1500 = overengineered garbage regarding the power supply,

    From MKRNB library only uses NBUDP and MODEM class since the rest is somewhat garbage.. Aggressive modem powersaving set in custom modem init/wakeup sequence.
    NB! Using APN mdatks and operator code Telenor, change in connectModem() function if needed.

    Power usage before tuning approx 31.4mA when awake/listening, peaks to 115mA on modem wakeup/transmit, and ca 2.2mA sleeping with spikes xx between 8s sleep loop.
    After misc tuning inc. compiler changes 29.8mA awake/listening, peak ca 115mA modem init,  0.7mA sleeping.

***********************************************************************/

//#define DEBUG  //enable USB serial debug (if not defined disables USB to save power).

#define SLEEPLOOPS 330 //ca *8s pr to get roughly 49min sleep, adjusted for clockdrift. Relay_254: 361
#define MY_NODE_ID 254 //<============================= Unique NodeID
#define BUFFERSIZE 256 //512 //Max struct array size for stored messages pr listen interval. Adjust to safely fit in available RAM

IPAddress serverIP(xxx, xxx, xxx, xxx); //udp server address
#define REMOTE_PORT 1234   //udp server port

#define MY_DEBUG // Enable debug prints to serial monitor
//#define MY_BAUD_RATE 115200
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_CE_PIN A1
#define MY_RF24_CS_PIN A2
//#define MY_RF24_IRQ_PIN A3
#define MY_GATEWAY_SERIAL //to force simple gateway mode, workaround for mobile forward
#define MY_DISABLED_SERIAL  //to skip serial wait. 
#include <MySensors.h>
#include <MKRNB.h>
#include <ArduinoLowPower.h>
#include <TemperatureZero.h>
#include <WDTZero.h> //watchdog (reset if modem hangs etc) and will force LowPower deepsleep to wake after 8s .

NBUDP udp;

byte wdtCounter, wdtCounterReset;
WDTZero MyWatchDoggy;

TemperatureZero TempZero = TemperatureZero();

unsigned long previousMillis; //used for sleep trigger
unsigned int hourCounter = 0;  //total hours since boot
unsigned long packetsCounter = 0; //total packets sent since boot
unsigned long bytesCounter = 0; //total bytes sent since boot
unsigned int packetsLastHourCounter = 0;

unsigned int sendUnsureCounter = 999; //high number forces reset/init on boot connect. Used to hard reset modem if many send fails, somewhat hacky watchdog.

//Prepare messagestack
typedef struct { //Based on MySensors protocol
    byte nodeID;
    byte sensorID;
    byte command;
    //byte ack; Will always be zero so hard coded to 0 in send below to save memory
    byte type;
    char payload[20]; //Acording to Mysensors Protocol max payload=25 bytes
} NRFPacketStruct;

NRFPacketStruct NRFPacket[BUFFERSIZE]; //static reserve amount.
unsigned int nextFreePointer = 0; //Pointer to next free packet array.


void setup() {
    initVariant();  //re-init some pin defaults (important fixup after mysensors init since messes with the pin settings)
#ifdef DEBUG
    MODEM.debug();
#else
    USBDevice.detach(); //Saves 0.5mA.
    //MODEM.debug();
#endif

    //Set digital pins to input to save on current drain. Saves 0.1mA
    for (int i = 0; i < 7; i++) {
        pinMode(i, INPUT_PULLUP);
    }
    for (int i = 11; i < 15; i++) { //pins 8-10 used by NRF MISO; MOSI; SCK
        pinMode(i, INPUT_PULLUP);
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); //blink to tell alive
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);

    //Set SARA control pins.
    pinMode(SARA_RTS, OUTPUT);
    digitalWrite(SARA_RTS, LOW); //serial flow control not available in HW, will always be forced low.

    pinMode(SARA_RESETN, OUTPUT); // NEVER EVER use RESET_N
    digitalWrite(SARA_RESETN, LOW); //inverted so high on SARA. 

    pinMode(SARA_PWR_ON, OUTPUT);
    digitalWrite(SARA_PWR_ON, LOW); //inverted so high on SARA

    // initialize serial communications and wait for port to open:
    //Serial.begin(115200); <started by mysensors
    //while (!Serial);

    SerialSARA.begin(115200);
    while (!SerialSARA);
    //Serial.println("Initial wait 10s to defend against zombies");
    delay(10000); //Blocking delay for zombie protection

    messageToBuffer(MY_NODE_ID, 255, 1, 14, "1"); //Add to slot 0 in LIFO receive buffer as a header
    //initial temp reading for first send to popuate.
    TempZero.init();
    delay(500);
    messageToBuffer(MY_NODE_ID, 255, 1, 0, String(TempZero.readInternalTemperature()));
    TempZero.disable(); //saves ~60uA in standby

    MyWatchDoggy.setup(WDT_HARDCYCLE16S); //WDT_SOFTCYCLE1M); //watchdog generates a reset if software loop does not clear WDT within every 1 min

    connectModem();  //init the modem and open UDP socket.

    readVoltage(); //Force 1 read since first on boot seems to fail/show max. Fix OK.
    //send directly/bypass buffer to show alive early on boot..
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/11_NBIoT-MKR1500"); //HW info
    delay(250);  //to not overload modem
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/12_7.0"); //Sketch version
    delay(250);  //to not overload modem
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());
    delay(1000);  //to ensure time for last transmit before shutting down modem

    powerOffModem();  //flush and close UDP socket, shut down modem

    previousMillis = millis();
} //==================END setup==================

void loop() {
    MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min it will reset
    yield();

    //NOTE. NRF interrupt listener adds to buffer, send are done in a single burst right before sleep

    //=============Sleep section================
    if (millis() - previousMillis > 660000) { //run after 11 min, holds in sleep for 49, then resets perviousMillis
        //if (millis() - previousMillis > 330000) { //DEBUG SEMI SPEEDRUN
        //if (millis() - previousMillis > 60000) { //DEBUG SPEEDRUN

        connectModem(); //init modem and open UDP socket

        digitalWrite(MY_RF24_CE_PIN, LOW);  //set NRF to output/low power mode to stop receive.

        //Add some relay statistics
        messageToBuffer(MY_NODE_ID, 255, 1, 24, String(hourCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 25, String(packetsCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 26, String(packetsLastHourCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 38, readVoltage());
        //1 more stat (bytes total) sendt outside buffer before end padding since aggregated while send.

        for (int i = 0; i < nextFreePointer; i++) {
            digitalWrite(MY_RF24_CE_PIN, LOW);  //Keep forcing NRF to output/low power mode (mysensors blob sometimes overrides this)
            sendFromBuffer(i);
            delay(250); //to not overload modem. 50 seems too low for bad reception. Testing 250
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min or it will reset the cpu.
        }
        nextFreePointer = 0; //Reset stack pointer.

        //Final modem statistic (bytes total) and end padding sent outside buffer
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/27_" + bytesCounter);
        delay(250);
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/14_0"); //end transmission message

        powerOffModem();  //flush and close UDP socket, shut down modem

        for (int i = 0; i < SLEEPLOOPS; i++) {  //Sleep Loop, 8s ca each.
            digitalWrite(MY_RF24_CE_PIN, LOW);  //Keep forcing NRF to output/low power mode, mysensors blob sometimes overrides this.
#ifdef DEBUG
            delay(8000);   Serial.print("z"); //blocking delay to simulate deep sleep.
#else
            LowPower.deepSleep(8000);  //8s max, will wake every 8s by watchdog timer set by WDTZero.
#endif
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min  it will reset the cpu.

            digitalWrite(LED_BUILTIN, HIGH); //flash to tell still alive
            delay(50); //do not use from LowPower here since disturbs 8s RTC timer.
            digitalWrite(LED_BUILTIN, LOW);
        }

        messageToBuffer(MY_NODE_ID, 255, 1, 14, "1"); //Add to slot 0 in FIFO receive buffer as a header
        //Read temp to buffer as soon as possible before CPU starts heating up.
        TempZero.init();
        delay(500);
        messageToBuffer(MY_NODE_ID, 255, 1, 0, String(TempZero.readInternalTemperature()));
        TempZero.disable(); //saves ~60uA in standby

        digitalWrite(MY_RF24_CE_PIN, HIGH); //enable NRF radio and start listening/filling buffer.

        hourCounter++;
        packetsLastHourCounter = 0;
        previousMillis = millis();

        //Serial.println("DEBUG End millis section");
    }

} //=============END loop=================

//===============functions below=========================
void connectModem() {
    MyWatchDoggy.clear();

    //Serial.println("Modem init");
    digitalWrite(LED_BUILTIN, HIGH); //turn on to indicate modem connecting

    digitalWrite(SARA_RTS, LOW); //serial flow control not available in HW but must be set low to communicate.

    digitalWrite(SARA_PWR_ON, HIGH); // Send Poweron _pulse (MKR pin high = low on SARA)
    delay(200); //original 150
    digitalWrite(SARA_PWR_ON, LOW);

    String response; //general AT catch response used multiple
    MODEM.send("AT" );  //just to pester modem to wake serial
    MODEM.waitForResponse(2000, &response);  wait(500);
    MODEM.send("AT" );  //just to pester modem to wake serial
    MODEM.waitForResponse(2000, &response);  wait(500);
    MyWatchDoggy.clear();
    if ( (sendUnsureCounter > 50) ) { //Software reset after multiple fail send.
        //initVariant();
        MODEM.send("AT+UPSV=4");  //Autosleep power saving based on UART TX line activity. Saves 7.5mA when idle/sleeping.
        MODEM.waitForResponse(500, &response);  wait(500);
        MODEM.send("AT\\Q0");  //  AT\Q0 - disable flow control. 1=software flow control
        MODEM.waitForResponse(500, &response);  wait(500);
        MODEM.send("AT+CEDRXS=0");  //use of eDRX disabled
        MODEM.waitForResponse(500, &response);  wait(500);
        MODEM.send("AT+CPSMS=1, , , \"00011100\",\"00000000\""); //Network sleep. https://www.u-blox.com/en/docs/UBX-18019856 for TAU values
        //MODEM.send("AT+CPSMS=0, , , \"00000000\",\"00000000\""); //Network sleep. https://www.u-blox.com/en/docs/UBX-18019856 for TAU values
        MODEM.waitForResponse(500, &response);  wait(500);
        MODEM.send("AT+CFUN=15"); //software reset  (with detach from network and saving of NVM parameters),
        MODEM.waitForResponse(500, &response);  wait(500);
        
        wait(4000); //wait for modem to get back from software reset (CFUN=15)
        MyWatchDoggy.clear();
        MODEM.send("AT" );  //just to pester modem after reset
        MODEM.waitForResponse(2000, &response);  wait(500);
        MODEM.send("AT" );  //just to pester modem to wake serial
        MODEM.waitForResponse(2000, &response);  wait(500);
        MODEM.send("AT" );  //just to pester modem to wake serial
        MODEM.waitForResponse(2000, &response);  wait(500);

        //======set operator parameters=====
        MODEM.send("AT+COPS=1,2,\"24201\""); // Operator Telenor
        MODEM.waitForResponse(500, &response);
        MODEM.send("AT+CGDCONT=1,\"IP\",\"mdatks\""); // Set APN to mdatks
        MODEM.waitForResponse(500, &response);

        messageToBuffer(MY_NODE_ID, 255, 1, 14, "2"); //Indicate a soft reset have happened

        sendUnsureCounter = 0;
    }

    //Serial.println("Enable mobile and check signal, CSQ must be below 99.99");
    MODEM.send("AT+CFUN=1"); // sets the MT to full functionality
    MODEM.waitForResponse(500, &response);   wait(1000);

    MyWatchDoggy.clear();
    //hold until modem attached to network (less than 99.99).
    String signalStrength = "99";
    //while (response == "+CSQ: 99,99") {
    while (signalStrength == "99") {
        MODEM.send("AT+CSQ");
        if (MODEM.waitForResponse(100, &response) == 1) {
            int firstSpaceIndex = response.indexOf(' ');
            int lastCommaIndex = response.lastIndexOf(',');

            if (firstSpaceIndex != -1 && lastCommaIndex != -1) {
                signalStrength = response.substring(firstSpaceIndex + 1, lastCommaIndex);
            }
        }
        wait(500);
    }

    MyWatchDoggy.clear();

    //test for registered on network
    while (response != "+CEREG: 0,1") {
        MODEM.send("AT+CEREG?");
        MODEM.waitForResponse(500, &response);
        wait(200);
    }

    //Activate and test GPRS
    MODEM.send("AT+CGATT=1");
    MODEM.waitForResponse(500, &response);
    while (response != "+CGACT: 1,1") {
        MODEM.send("AT+CGACT?");
        MODEM.waitForResponse(500, &response);
        wait(200);
    }

    MODEM.send("AT+CMGF=1"); //message format to text mode
    MODEM.waitForResponse(500, &response);  wait(500);
    MODEM.send("AT+UDCONF=1,1"); // HEX mode enabled
    MODEM.waitForResponse(500, &response);  wait(500);

    //A bit hacky. Force-test that modem gives us an open port before giving udp.begin() a chance. Will be closed in powerOffModem. If not will hold here unttil watchdog resets 
    while (response != "+USOCR: 0") {
        MODEM.send("AT+USOCR=17,0"); //Grab 0 as testing port
        MODEM.waitForResponse(500, &response);
        wait(500);
    }
    MyWatchDoggy.clear();

    udp.begin(5000); //Open modem UDP socket. Will select port 1 since 0 is taken as testing port
    wait(500);

    digitalWrite(LED_BUILTIN, LOW); //turn off to indicate modem OK
    //Serial.println("Modem connected");
} //END connectModem

void powerOffModem() {
    MyWatchDoggy.clear();
    delay(500); //give modem some chance to recover before closing socket (else might ignore AT command which creates problems in next connect..)
    udp.stop();
    delay(500);
    //Hacky, close testing and potential hanging UDP sockets . If already OK will return "+CME ERROR: Operation not allowed"
    MODEM.send("AT+USOCL=0");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=1");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=2");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=3");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=4");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=5");
    MODEM.waitForResponse(500);
    MODEM.send("AT+USOCL=6");
    MODEM.waitForResponse(500);

    //note holdovers from full shutdown of modem. not worth it/higher power expense.
    //  MODEM.send("AT+CPWROFF" );  //TESTING also try CPWROFF
    //  MODEM.waitForResponse(500);
    //Logic high on Arduino pin = logic low on SARA R4
    /*    pinMode(SARA_PWR_ON, OUTPUT);
        digitalWrite(SARA_PWR_ON, HIGH);
        delay(2000); //over 1.5s is shut down
        pinMode(SARA_PWR_ON, INPUT);  //Floating since, SARA has internal pullup*/
}


void receive(const MyMessage &message)  {//Kicked off by mysensors lib by interrupt. Extracts from mysensors and forwards to messageToBuffer() which puts mesasge in FIFO stack.
    messageToBuffer(message.sender, message.sensor, message.getCommand(), message.type, String(message.getFloat()).c_str());
    //note: ack from mysensors protocol ignored since only support 0 which are hardcoded later in send

    //flash led to indicate receive
    digitalWrite(LED_BUILTIN, HIGH);
    wait(30);
    digitalWrite(LED_BUILTIN, LOW);

    packetsCounter++;
    packetsLastHourCounter++;
} //END mysensors message receive events handler

void messageToBuffer(byte nodeID, byte sensorID, byte command, byte type, String payload) {  //add message to stack and move pointer
    if (nextFreePointer < BUFFERSIZE) {
        NRFPacket[nextFreePointer].nodeID = nodeID;
        NRFPacket[nextFreePointer].sensorID = sensorID;
        NRFPacket[nextFreePointer].command = command;
        //byte ack in MySensors protocol; Always zero here so hard coded in message reconstruct before mobile send
        NRFPacket[nextFreePointer].type = type;
        payload.toCharArray(NRFPacket[nextFreePointer].payload, 20); //copy char array input

        nextFreePointer++;
        //Serial.print("Message added to stack, pointer moved to: ");
        //Serial.println(nextFreePointer);
    }  /*else { //buffer full and silently drop packet
        //Serial.print("Buffer full, stackPointer: ");
        //Serial.println(nextFreePointer);.
    }*/
}

boolean sendFromBuffer(unsigned int bufferID) {  //format message string from buffer row and kick off send via mobile by udpSendString() function, then clear.
    //construct message string: Based on MySensors protocol with nodeID@ prefix to allow for separation at receiver.
    String message = String(MY_NODE_ID);
    message += "@";
    message += NRFPacket[bufferID].nodeID;
    message += "/";
    message += NRFPacket[bufferID].sensorID;
    message += "/";
    message += NRFPacket[bufferID].command;
    message += "/0/"; // ack from MySensors protocol; Will always zero so hard coded here.
    message += NRFPacket[bufferID].type;
    message += "_";
    message += NRFPacket[bufferID].payload;
    //Serial.print("DEBUG CONSTRUCT TO SEND: ");
    //Serial.println(message);

    udpSendString(serverIP, REMOTE_PORT, message);

    //reset current buffer slot as a security step.
    NRFPacket[bufferID].nodeID = 0;
    NRFPacket[bufferID].sensorID = 0;
    NRFPacket[bufferID].command = 0;
    NRFPacket[bufferID].type = 0;
    NRFPacket[bufferID].payload[0] = '/0';
}  //END sendFromBuffer()

void udpSendString(IPAddress udpServer, unsigned int udpServerport, String message) {  //used from loop to send from msgBuffer stack. returns sent bytes or 0 if failed.
    boolean packetSuccess = 0;
    int sentBytes = 0;

    udp.beginPacket(udpServer, udpServerport);
    //sentBytes = udp.print(message); //udp.print nicely converts string to hex before send
    sentBytes = udp.print(message); //udp.print nicely converts string to hex before send
    packetSuccess = udp.endPacket(); //0 = failed. NB: might still be successful if first packet after modem 30s+ idle,

    if (packetSuccess == 0) { //packet failed (NB: might still be sent OK if first packet after long idle).
        //Serial.print("Send unsure, bytes: "); Serial.println(sentBytes); //verify on server side if was successful, usually are..
        sendUnsureCounter++;
        delay(200); //give modem some time to wake up if was first packet in a while
    } else {
        //Serial.print("Send OK, bytes: "); Serial.println(sentBytes);
        sendUnsureCounter = 0; //reset failsafes
        // hardResetModem = 0; //reset failsafes
    }
    bytesCounter += sentBytes; //update global byte statistics
}

String readVoltage() {
    int adcReading = analogRead(ADC_BATTERY);
    // convert to a voltage:
    float batteryVoltage = (adcReading / 1024.0) * 4.2;
    //reading = (reading / 1024) * 3.27; //Convert to actual voltage vs. reference.
    return String(batteryVoltage, 2);
}
