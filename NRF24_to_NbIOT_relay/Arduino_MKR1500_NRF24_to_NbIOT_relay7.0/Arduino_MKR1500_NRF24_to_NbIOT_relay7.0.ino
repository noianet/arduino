/***********************************************************************
    Board: Arduino MKR 1500 with Ubox SARA R4

    Modified Mysensors serial gateway for raw udp over mobile NBIiT/LTE-M.
    Buffer stack to store incoming data

    Note: Only reads mysensors float values. Incoming NRF packets are stored via interrupt in the message stack awaiting transmit loop pickup.
    Listens for 11 minutes, then sleeps 49 (NRF sensor nodes runs at a 10min transmit interval)

    Mostly uses unblocking wait() and smartsleep() from mysensors library whikle awake which allow for NRF interrupts/store to buffer while idling.
    Received NRF packets are stored in a FIFO buffer while awake. Then wakes mobile and sends all just before sleep routine .
    Sends a xx/14 "1" header to inticate start FIFO buffer, and ends with relay statistics and a xx/14 "0" to indicate end transmission.
    Modem sleeps as much as possible, only wakes right before sleep, burst send from FIFO, and then put to sleep before microcontroller 49min sleep routine.

    LED flashes on NRF packet receive while awake, and constant light while modem conect. In 49min sleep routine flashes every 8s to indicate still alive.

    Hardware watchdog routine  resets micro  if code freeze (incl. should caktch some while() connecting loops if faulty modem).
    In setup does an ugly hardware reset (to be avoided according to SARA datasheet) but sometimes necessary to fix a hard modem hang.

    Misc notes:
    V7.x Rewrite for MKR 1500, removed TelenorIOT library and uses standard udp, and removed a lot of string buffers etc. New feature sends statistics for total bytes sent since boot with a /nodeID/1/0/27 message.

    NBNB! Always run with a connected battery to avoid brownout crashes.... And seems important to connect battery first, then USB (some stupid battery protection in HW?).
    Needs external battery charger since onboard limited to <0.1A and 4hr max chargetime....Seems like MKR 1500 = overengineered garbage regarding the power supply,

    From MKRNB library only uses NBUDP (and therfore MODEM class) since the rest is utter garbage and crashes/locks up the modem..
    Modem put to graceful network disconect/sleep with AT+CPWROFF, and wake by pulling SARA_PWR_ON low for 1 sec. (pin only signal/does not switch power supply to modem).

    Power usage: approx 32.0mA when awake/listening, peaks to 115mA on modem wakeup/transmit, and ca 2.8mA sleeping. 

***********************************************************************/
#include <MKRNB.h>
#include <ArduinoLowPower.h>
#include <WDTZero.h> //watchdog (reset if modem hangs etc) and will force LowPower deepsleep to wake after 8s .

//#define DEBUG  //enable USB serial debug (othervisw disables USB to save power).
//#define ATDEBUG  //enable debug display AT commands. Must enable DEBUG as well for this

#define SLEEPLOOPS 361 // ca *8s pr to get roughly 49min sleep, adjusted for clockdrift. Relay_254: 361
#define MY_NODE_ID 254 //<============================= Unique NodeID
#define BUFFERSIZE 512 //Max struct array size for stored messages pr listen interval. Adjust to safely fit in available RAM

IPAddress serverIP(xxx, xxx, xxx, xxx); //udp server address
#define REMOTE_PORT 1234   //udp server port

//#define MY_DEBUG // Enable debug prints to serial monitor
#define MY_BAUD_RATE 115200
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_CE_PIN 0
#define MY_RF24_CS_PIN 1
#define MY_RF24_IRQ_PIN 2
#define MY_GATEWAY_SERIAL //to force simple gateway mode, workaround for mobile forward
#define MY_DISABLED_SERIAL  //to skip serial wait. 
#include <MySensors.h>

NBUDP udp;

byte wdtCounter, wdtCounterReset;
WDTZero MyWatchDoggy; 

unsigned long previousMillis; //used for sleep trigger
unsigned int hourCounter = 0;  //total hours since boot
unsigned long packetsCounter = 0; //total packets sent since boot
unsigned long bytesCounter = 0; //total bytes sent since boot
unsigned int packetsLastHourCounter = 0;

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
#if !defined(DEBUG)
    USBDevice.detach(); // detach USB . Saves 0.5mA. NBNBNB: Comment out for serial debug
#endif

    //Set digital pins to input  to save on current drain. Saves 0.1mA
    for (int i = 3; i < 7; i++) { //pins 0-2 used by NRF CE, CS, IRQ
        pinMode(i, INPUT);
    }
    for (int i = 11; i < 15; i++) { //pins 8-10 used by NRF MISO; MOSI; SCK
        pinMode(i, INPUT);
    }

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH); //blink to tell alive
    wait(100);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(SARA_PWR_ON, OUTPUT);
    digitalWrite(SARA_PWR_ON, HIGH);
    // reset the SerialSARA module <- THIS IS NOT IDEAL but sometimes only option to get an unresponsive modem back (watchdog should reset CPU if modem hang).
    pinMode(SARA_RESETN, OUTPUT);
    digitalWrite(SARA_RESETN, HIGH);
    delay(100);
    digitalWrite(SARA_RESETN, LOW);

    // initialize serial communications and wait for port to open:
    //Serial.begin(115200); <started by mysensors
    //while (!Serial);

    Serial.println("Initial wait 10s to defend against zombies");
    delay(10000); //Blocking delay for zombie protection

    MyWatchDoggy.setup(WDT_SOFTCYCLE2M); //watchdog generates a reset if software loop does not clear WDT within every 2 min

    MODEM.begin(115200);  //modem class from MKRNB, there for NBUDP.
    while (!SerialSARA);

    connectModem();  //init the modem. 

    readVoltage(); //Force 1 read since first on boot seems to fail/show max. Fix OK.
    //send directly/bypass buffer to show alive early on boot.
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/11_NBIoT-MKR1500"); //HW info
    wait(50); //to not overload modem
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/12_7.0"); //Sketch version
    wait(50); //to not overload modem
    udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());
    wait(1000); //to ensure time for last transmit before shutting down modem
    sendATcommand("AT+CPWROFF" , 500); //modem shutdown

    messageToBuffer(MY_NODE_ID, 255, 1, 14, "1"); //Add to slot 0 in LIFO receive buffer as a header

    previousMillis = millis();
} //==================END setup==================

void loop() {
    MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min it will reset
    LowPower.idle(1000); //waits for NRF interrupt or timeout

    //Note. NRF interrupt listener adds to buffer, send are done in a single burst right before sleep

    //=============Sleep section================
   if (millis() - previousMillis > 660000) { //run every 11 min, holds in sleep for 49
     //if (millis() - previousMillis > 30000) { //DEBUG SPEEDRUN
        Serial.println("DEBUG entering modem connect");
        digitalWrite(SARA_PWR_ON, LOW); //power pin low 1 sec to wake from powered down state
        LowPower.idle(1000);
        digitalWrite(SARA_PWR_ON, HIGH);
        LowPower.idle(1000);
        //pester modem with a few AT to wake it up, then connect routine.
        sendATcommand("AT" , 1000); //wake modem by pestering it with some UART TX activity
        sendATcommand("AT" , 500);
        sendATcommand("AT" , 500);
        sendATcommand("AT" , 500);
        connectModem();

        digitalWrite(MY_RF24_CE_PIN, LOW);  //set NRF to output/low power mode to stop receive.

        //Add some relay statistics as padding end of buffer
        messageToBuffer(MY_NODE_ID, 255, 1, 24, String(hourCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 25, String(packetsCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 26, String(packetsLastHourCounter));
        messageToBuffer(MY_NODE_ID, 255, 1, 38, readVoltage());
        // messageToBuffer(MY_NODE_ID, 255, 1, 27, String(bytesCounter));  <-- these two sent outside buffer further down.
        //messageToBuffer(MY_NODE_ID, 255, 1, 14, "0"); //Indicate last packet from stack sequence

        //Send from buffer
        Serial.println("DEBUG entering modem sendloop");
        for (int i = 0; i < nextFreePointer; i++) {
            digitalWrite(MY_RF24_CE_PIN, LOW);  //Keep forcing NRF to output/low power mode (mysensors blob sometimes overrides this)
            delay(50); //to not overload modem
            sendFromBuffer(i);
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min or it will reset the cpu.
        }
        nextFreePointer = 0; //Reset stack pointer.

        //Since bytes are counted during in transmit each send from buffer, final tally are sendt outside loop/buffer at the end. This one and the end transmission message bytes are addad to the total and reported in the next go-around..
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/27_" + bytesCounter);
        delay(50);
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/14_0"); //end transmission message

        Serial.println("DEBUG entering sleep section");
        LowPower.idle(1000); //Just to ensure final packet are sent before shutting down modem
        sendATcommand("AT + CPWROFF" , 500); //modem shutdown
        
        MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min  it will reset the cpu.
        
        for (int i = 0; i < SLEEPLOOPS; i++) {  //Sleep Loop, 8s ca each.
            digitalWrite(MY_RF24_CE_PIN, LOW);  //Keep forcing NRF to output/low power mode, mysensors blob sometimes overrides this.
            LowPower.deepSleep();  //will wake every 8s by watchdog timer set by WDTZero.
            //delay(8000); Serial.print("z");
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min  it will reset the cpu.
            digitalWrite(LED_BUILTIN, HIGH); //flash to tell still alive
            delay(50); //do not use from LowPower here since disturbs 8s RTC timer.
            digitalWrite(LED_BUILTIN, LOW);
        }
        Serial.println();
        Serial.println("End sleep section, ");

        messageToBuffer(MY_NODE_ID, 255, 1, 14, "1"); //Add to slot 0 in FIFO receive buffer as a header
        digitalWrite(MY_RF24_CE_PIN, HIGH); //enable NRF radio and start listening/filling buffer.

        hourCounter++;
        packetsLastHourCounter = 0;
        previousMillis = millis();
    }

} //=============END loop=================

//===============functions below=========================

void receive(const MyMessage &message)  {//Kicked off by mysensors lib by interrupt. Extracts from mysensors and forwards to messageToBuffer() which puts mesasge in FIFO stack.
    messageToBuffer(message.sender, message.sensor, message.getCommand(), message.type, String(message.getFloat()).c_str());
    //note: ack from mysensors protocol ignored since only support 0 which are hardcoded later in send
} //END mysensors message receive events handler

void messageToBuffer(byte nodeID, byte sensorID, byte command, byte type, String payload) { //char payload[20]) {  //add message to stack and move pointer
    if (nextFreePointer < BUFFERSIZE) { 
        NRFPacket[nextFreePointer].nodeID = nodeID;
        NRFPacket[nextFreePointer].sensorID = sensorID;
        NRFPacket[nextFreePointer].command = command;
        //byte ack in MySensors protocol; Always zero here so hard coded in message reconstruct before mobile send
        NRFPacket[nextFreePointer].type = type;
        payload.toCharArray(NRFPacket[nextFreePointer].payload, 20); //copy char array input

        nextFreePointer++;
        Serial.print("Message added to stack, pointer moved to: ");
        Serial.println(nextFreePointer);
    } else { //buffer full and silently drop packet
        Serial.print("Buffer full, stackPointer: ");
        Serial.println(nextFreePointer);
    }

    //flash led to indicate receive
    digitalWrite(LED_BUILTIN, HIGH);
    wait(30);
    digitalWrite(LED_BUILTIN, LOW);

    packetsCounter++;  //Always add here to indicate if more than buffer allows
    packetsLastHourCounter++;
}

boolean sendFromBuffer(unsigned int bufferID) {  //format message string from buffer row and kick off send via mobile by udpSendString() function, then clear.
    //construct message string: Based on MySensors protocol with nodeID@ prefix to allow for separation at receiver.
    String   message = String(MY_NODE_ID);
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
    sentBytes = udp.print(message); //udp.print nicely converts string to hex before send
    packetSuccess = udp.endPacket(); //0 = failed. NB: might still be successful if first packet after modem 30s+ idle,

    if (packetSuccess == 0) { //packet failed (NB: might still be sent OK if first packet after long idle).
        Serial.print("Send unsure, bytes: "); Serial.println(sentBytes); //verify on server side if was successful, usually are..
        //sendUnsureCounter++;
        wait(200); //give modem some time to wake up if was first packet in a while
    } else {
        Serial.print("Send OK, bytes: "); Serial.println(sentBytes);
        //sendUnsureCounter = 0;
    }
    bytesCounter += sentBytes; //update global byte statistics
}

void connectModem() {
    MyWatchDoggy.clear();
    digitalWrite(LED_BUILTIN, HIGH); //turn on to indicate modem connecting

    sendATcommand("AT" , 1000); //wake modem
    //Serial.println("Ensure radio disabled");
    /*  sendATcommand("AT + COPS = 2" , 500);  //ensure deregistered from network
        sendATcommand("AT + IPR = 115200", 500); //lock UART to 115200 baud
        sendATcommand("AT + UPSV = 4" , 500);  //autosleep power saving based on UART TX line activity
        sendATcommand("AT + CPSMS = 0, , , \"00000000\",\"00000000\"" , 500); //Disabling network sleep since using airplane mode during hibernate instead.
        sendATcommand("AT+CFUN=15" , 500); //silent reset  (with detach from network and saving of NVM parameters),*/
    Serial.println("Enable mobile and check signal, CSQ must be below 99.99");
    sendATcommand("AT+CFUN=1" , 500); // sets the MT to full functionality
    sendATcommand("AT+CSQ" , 2000);  //check signal strength a few times to nag modem/ensure radio is awake.
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CMGF=1" , 500); //message format to text mode
    sendATcommand("AT+UDCONF=1,1" , 500); // HEX mode enabled
    /*  Serial.println("Set operator/apn");
        sendATcommand("AT+COPS=1,2,\"24201\"" , 500);
        sendATcommand("AT+CGDCONT=1,\"IP\",\"mdatks\"" , 500);  //set APN
        sendATcommand("AT+UAUTHREQ=1,0" , 500); //Configures the authentication parameters of a defined PDP/EPS bearer
        sendATcommand("AT+CGDCONT?" , 500); //redudnant but just extra check.*/
    Serial.println("Testing connected to mobile network");
    while (true) {
        String ATresult = String(sendATcommand("AT+CEREG?" , 500).substring(20, 23));
        //Serial.println("DEBUG substring:" + ATresult);
        if (ATresult == "0,1") { // or ATresult == "0,2")
            Serial.println("Success, status:" + ATresult);
            break;
        } else {
            Serial.println("Still connecting, status:" + ATresult);
            wait(2000);
        }
    }
    //    Serial.print("Connected, assigned PSM values:");
    //    Serial.println(sendATcommand("AT+UCPSMS?" , 500));  //prints assigned PSM valuwes from netwiork

    MyWatchDoggy.clear();
    Serial.println("Activate and test GPRS ");
    sendATcommand("AT+CGATT=1" , 500);
    while (true) {
        String ATresult = String(sendATcommand("AT+CGACT?" , 500).substring(12, 21)); 
        if (ATresult == "+CGACT: 1") {
            Serial.println("Success, status:" + ATresult);
            break;
        } else {
            Serial.println("GPRS error, status:" + ATresult);
            wait(2000);
        }
    }
    MyWatchDoggy.clear();
    Serial.println("Open UDP socket");
    udp.begin(5000); //this part uses sandard MKNRB lib since has some convenient string conversion stuff.
    digitalWrite(LED_BUILTIN, LOW); //turn off to indicate modem OK
    Serial.println("Modem connected");
} //END connectModem

String sendATcommand(String command, unsigned long timeout) {
    String ATresponse = "";
    SerialSARA.println(command);
    unsigned long lastRead = millis();   // last time a char was available
    while (millis() - lastRead < timeout) {
        while (SerialSARA.available()) {
            char c = SerialSARA.read();
            ATresponse += c;  // append to the result string
            lastRead = millis();   // update the lastRead timestamp
        }
    }
#ifdef ATDEBUG
    Serial.println(ATresponse);
#endif
    return ATresponse;
}

String readVoltage() {
    int adcReading = analogRead(ADC_BATTERY);
    // convert to a voltage:
    //float batteryVoltage = adcReading * (4,2 / 1023.0);
    float batteryVoltage = (adcReading / 1024.0) * 4.2;
    //reading = (reading / 1024) * 3.27; //3.28 Convert to actual voltage vs. reference.
    return String(batteryVoltage, 2);
}

