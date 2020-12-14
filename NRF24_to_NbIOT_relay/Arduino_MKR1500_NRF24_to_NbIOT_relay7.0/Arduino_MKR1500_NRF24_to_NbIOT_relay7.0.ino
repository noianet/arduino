/***********************************************************************
    Board: Arduino MKR 1500 with Ubox SARA R4

    Modified Mysensors serial gateway for raw udp over mobile NBIiT/LTE-M.
    10 slot buffer stack to store incoming data since mobile unreliable/often needs reconnect to network after a few minutes idling.

    Note: Only reads mysensors float values. Incoming NRF packets are stored via interrupt in the message stack awaiting transmit loop pickup.
    Listens for 11 minutes, then sleeps 49 (NRF sensor nodes runs at a 10min transmit interval)

    Mostly uses unblocking wait() and smartsleep() from mysensors library which are equal to delay()

    V7.x Rewrite for MKR 1500, removed TelenorIOT library and uses standard udp. New feature sends statistics for total bytes sent since boot with a /nodeID/1/0/27 message.
    Fast LED flash on NRF packet receive and slow'ish blink once a minute when sleeping to indicate not dead.

    NBNB! Always run with a connected battery to avoid brownout crashes.... And seems important to connect battery first, then USB (some stupid battery protection in HW?).
    Add external battery charger since onboard limited to <0.1A and 4hr max chargetime....Seems like MKR 1500 = overengineered garbage regarding the power supply,

    Power usage: approx 33mA when awake/listening, peaks to 115mA on transmit, and ca 1mA sleeping.

    Note, workaround if long term lockups..  https://github.com/arduino-libraries/MKRGSM/issues/66

    Only NBUDP (and therfore MODEM class) used from MKRNB since rest is utter garbage and crashes/locks up  the modem.
    Using modem powersave instead of switching PWR pin etc. from MKRNB sketches (which crashes the modem often).

***********************************************************************/
#include <MKRNB.h>
#include <ArduinoLowPower.h>
#include <WDTZero.h> //watchdog frozen (modem hangs) and deepsleep 8s wakeup.
//#define DEBUG  //enable debug display AT commands.

#define SLEEPLOOPS 351 // ca *8s pr to get roughly 49min sleep, adjusted for clockdrift. Relay_254: 358
#define MY_NODE_ID 254 //<============================= Unique NodeID

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
WDTZero MyWatchDoggy; //generates a reset if software loop does not clear WDT on time (generally modem issues).

unsigned long previousMillis; //used for sleep trigger
unsigned int hourCounter = 0;  //total hours since boot
unsigned long packetsCounter = 0; //total packets sent since boot
unsigned long bytesCounter = 0; //total bytes sent since boot
unsigned int packetsLastHourCounter = 0;

//simple messagestack
String msgBuffer0 = "";
String msgBuffer1 = "";
String msgBuffer2 = "";
String msgBuffer3 = "";
String msgBuffer4 = "";
String msgBuffer5 = "";
String msgBuffer6 = "";
String msgBuffer7 = "";
String msgBuffer8 = "";
String msgBuffer9 = "";


void setup() {
    digitalWrite(LED_BUILTIN, HIGH); //blink to tell alive
    wait(100);
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(SARA_PWR_ON, OUTPUT);
    digitalWrite(SARA_PWR_ON, HIGH);
    // reset the SerialSARA module <- THIS IS NOT IDEAL.. But sometimes only option to get an unresponsive modem back (watchdog should reset).
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

    connectModem();  //init the modem. DO NOT USE POWER/RESET PIN FROM ARDUINO EXAMPLES.

    //Add some boot messages to buffer, will be picked up and sent from loop
    readVoltage(); //Force 1 read since first on boot seems to fail/show max. Fix OK.
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/11_NBIoT-MKR1500"); //HW info
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/12_7.0"); //Sketch version
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_1"); //Indicate gateway ready
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());

    previousMillis = millis();
} //==================END setup==================

void loop() {
    MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min it will reset.
    //Send from stack if not empty, else smartsleep for a bit. A bit simple mut not much to gain from a loop, and always assume send successful.
    if (msgBuffer0 != "") {
        //Serial.println("s0");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer0);
        msgBuffer0 = ""; //clear
    } else if (msgBuffer1 != "") {
        //Serial.println("s1");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer1);
        msgBuffer1 = ""; //clear
    } else if (msgBuffer2 != "") {
        //Serial.println("s2");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer2);
        msgBuffer2 = ""; //clear
    } else if (msgBuffer3 != "") {
        //Serial.println("s3");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer3);
        msgBuffer3 = ""; //clear
    } else if (msgBuffer4 != "") {
        //Serial.println("s4");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer4);
        msgBuffer4 = ""; //clear
    } else if (msgBuffer5 != "") {
        //Serial.println("s5");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer5);
        msgBuffer5 = ""; //clear
    } else if (msgBuffer6 != "") {
        //Serial.println("s6");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer6);
        msgBuffer6 = ""; //clear
    } else if (msgBuffer7 != "") {
        //Serial.println("s7");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer7);
        msgBuffer7 = ""; //clear
    } else if (msgBuffer8 != "") {
        //Serial.println("s8");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer8);
        msgBuffer8 = ""; //clear
    } else if (msgBuffer9 != "") {
        //Serial.println("s9");
        udpSendString(serverIP, REMOTE_PORT, msgBuffer9);
        msgBuffer9 = ""; //clear
    } else {  //buffer empty, smartsleep untill NRF interrupt or max timer
        smartSleep(2, CHANGE, 1000); //interrupt pin, interrupt type, wait in ms. Wake if nrf get packet.
    }

    //=============Sleep section================
    if (millis() - previousMillis > 660000) { //run every 11 min, holds in sleep for 49
        //if (millis() - previousMillis > 30000) { //DEBUG SPEEDRUN
        digitalWrite(MY_RF24_CE_PIN, LOW);  //set NRF to output/low power mode
        //Serial.println("DEBUG: Entering millis" );
        //Send statistics and bye messages directly/bypass buffer since we are outside stack handler above.
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/24_" + String(hourCounter));
        wait(50); //to not overload modem
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/25_" + String(packetsCounter));
        wait(50); //to not overload modem
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/26_" + String(packetsLastHourCounter));
        wait(50); //to not overload modem
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/27_" + String(bytesCounter));
        wait(50); //to not overload modem
        String currVoltage = readVoltage();
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + currVoltage);
        wait(50); //to not overload modem
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_0"); //Indicate gateway sleeping

        Serial.println("DEBUG entering sleep section");
        //COMMENT: No command for modem sleep needed since In connectModem() set auto-sleeps if no UART TX.

        MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min or it will reset the cpu.
        for (int i = 0; i < SLEEPLOOPS; i++) {  //Sleep Loop, 8s ca each.
            digitalWrite(MY_RF24_CE_PIN, LOW);  //DEBUG: force NRF to output/low power mode. Fighting MySensors blobs.
            LowPower.deepSleep();  //wake every 8s by watchdog timer
            //delay(8000);
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min or it will reset the cpu.
            digitalWrite(LED_BUILTIN, HIGH); //flash to tell still alive
            wait(50);
            //Serial.print("z");
            digitalWrite(LED_BUILTIN, LOW);
        }
        Serial.println();
        Serial.println("End sleep section,");
        //_________wake up radios_________
        //wake modem by pestering it with some UART TX activity
        sendATcommand("AT" , 1000);
        sendATcommand("AT" , 1000);
        sendATcommand("AT" , 1000);

        digitalWrite(MY_RF24_CE_PIN, HIGH); //enable NRF radio

        sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_1"); //Indicate gateway ready
        hourCounter++;
        packetsLastHourCounter = 0;
        previousMillis = millis();
    }
} //=============END loop=================

//===============functions below=========================
void connectModem() {
    //reset all deadman switches
    MyWatchDoggy.clear();
    digitalWrite(LED_BUILTIN, HIGH); //turn on to indicate modem connecting

    sendATcommand("AT" , 1000); //wake modem
    //Serial.println("Ensure radio disabled");
    sendATcommand("AT+COPS=2" , 500);  //ensure deregistered from network
    sendATcommand("AT+IPR=115200", 500); //lock UART to 115200 baud
    sendATcommand("AT+UPSV=4" , 500);  //autosleep power saving based on UART TX line activity
    //sendATcommand("AT+CPSMS=1,,,\"11111111\",\"00000000\"" , 500);  //"Never" wake to check for downstream data.
    //sendATcommand("AT+CPSMS=1,,,\"01100000\",\"00000000\"" , 500);  //Slow down wake to check for downstream data.
    sendATcommand("AT+CPSMS=0,,,\"00000000\",\"00000000\"" , 500);  //Seems like disabled is only way to avoid crashes.
    sendATcommand("AT+CFUN=15" , 500); //silent reset  (with detach from network and saving of NVM parameters),
    Serial.println("Enable radio and check signal, CSQ must be below 99.99");
    sendATcommand("AT+CFUN=1" , 500); // sets the MT to full functionality
    sendATcommand("AT+CSQ" , 2000);  //check signal strength a few times to nag modem/ensure radio is awake.
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CSQ" , 2000);
    sendATcommand("AT+CMGF=1" , 500); //message format to text mode
    sendATcommand("AT+UDCONF=1,1" , 500); // HEX mode enabled
    /*    Serial.println("Set operator/apn");
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
            delay(2000);
        }
    }
    Serial.print("Connected, assigned PSM values:");
    Serial.println(sendATcommand("AT+UCPSMS?" , 500));  //prints assigned PSM valuwes from netwiork

    MyWatchDoggy.clear();
    Serial.println("Activate and test GPRS ");
    sendATcommand("AT+CGATT=1" , 500);
    while (true) {
        String ATresult = String(sendATcommand("AT+CGACT?" , 500).substring(12, 21)); //CGATT
        if (ATresult == "+CGACT: 1") {
            Serial.println("Success, status:" + ATresult);
            break;
        } else {
            Serial.println("GPRS error, status:" + ATresult);
            delay(2000);
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
    // No need for extra line feed since most responses contain them anyways
#ifdef DEBUG
    Serial.println(ATresponse);
#endif
    return ATresponse;
}

void udpSendString(IPAddress udpServer, unsigned int udpServerport, String message) {  //used from loop to send from msgBuffer stack. returns sent bytes or 0 if failed.
    boolean packetSuccess = 0;
    int sentBytes = 0;
    sendATcommand("AT" , 100); //ping to wake modem if autosleep

    udp.beginPacket(udpServer, udpServerport);
    sentBytes = udp.print(message); //udp.print nicely converts string to hex before send
    packetSuccess = udp.endPacket(); //0 = failed. NB: might still be successful if first packet after modem 30s+ idle,

    if (packetSuccess == 0) { //packet failed (NB: might still be sent OK if first packet after long idle).
        Serial.print("Send unsure, bytes:"); Serial.println(sentBytes); //verify on server side if was successful, usually are..
        //sendUnsureCounter++;
        wait(200); //give modem some time to wake up if was first packet in a while
    } else {
        Serial.print("Send OK, bytes:"); Serial.println(sentBytes);
        //sendUnsureCounter = 0;
    }
    bytesCounter += sentBytes; //update global byte statistics
    //Serial.print("DEBUG end from udpSendString client.ready:"); Serial.print(client.ready()); Serial.print(", nbaccess.ready:"); Serial.println(nbAccess.ready());
} //END udpSendString

void receive(const MyMessage & message)  { //Kicked off by mysensors lib by interrupt. Unpacks and builds message string and forwards to sendMessage() function
    String msgValue = String(message.getFloat());
    //  if (!msgValue) {msgValue=String(message.getLong());}

    String messageConstruct = String(MY_NODE_ID)  + "@" + String(message.sender) + "/" + String(message.sensor) +  "/" + String(message.getCommand()) +  "/" + String(message.isAck()) +  "/" + String(message.type) +  "_" + msgValue;
    //Serial.println("messageConstruct: " + messageConstruct);
    packetsCounter++;
    packetsLastHourCounter++;
    sendMessage(messageConstruct);
    //flash led to indicate receive
    digitalWrite(LED_BUILTIN, HIGH);
    wait(30);
    digitalWrite(LED_BUILTIN, LOW);
} //END mysensors message receive events handler

void sendMessage(String payload) { //only adds to send buffer, sending from stack are done in loop.
    if (msgBuffer0 == "") {
        //Serial.println("a0");
        msgBuffer0 = payload;
    } else if (msgBuffer1 == "") {
        //Serial.println("a1");
        msgBuffer1 = payload;
    }  else if (msgBuffer2 == "") {
        //Serial.println("a2");
        msgBuffer2 = payload;
    }  else if (msgBuffer3 == "") {
        //Serial.println("a3");
        msgBuffer3 = payload;
    }  else if (msgBuffer4 == "") {
        //Serial.println("a4");
        msgBuffer4 = payload;
    }  else if (msgBuffer5 == "") {
        //Serial.println("a5");
        msgBuffer5 = payload;
    }  else if (msgBuffer6 == "") {
        //Serial.println("a6");
        msgBuffer6 = payload;
    }  else if (msgBuffer7 == "") {
        //Serial.println("a7");
        msgBuffer7 = payload;
    }  else if (msgBuffer8 == "") {
        //Serial.println("a8");
        msgBuffer8 = payload;
    }  else if (msgBuffer9 == "") {
        //Serial.println("a9");
        msgBuffer9 = payload;
    } else {
        Serial.println("DEBUG ALERT BUFFER FULL");
        wait(1); //symbolic wait to remember the packet that was lost to time.
    }
} //END sendMessage

String readVoltage() {
    int adcReading = analogRead(ADC_BATTERY);
    // convert to a voltage:
    //float batteryVoltage = adcReading * (4,2 / 1023.0);
    float batteryVoltage = (adcReading / 1024.0) * 4.2;
    //reading = (reading / 1024) * 3.27; //3.28 Convert to actual voltage vs. reference.
    return String(batteryVoltage, 2);
}


