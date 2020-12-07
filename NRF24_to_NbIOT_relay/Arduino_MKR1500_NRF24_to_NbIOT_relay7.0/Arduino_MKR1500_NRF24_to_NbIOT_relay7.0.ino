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
***********************************************************************/
#include <MKRNB.h>
#include <ArduinoLowPower.h>
#include <WDTZero.h> //watchdog frozen (modem hangs) and deepsleep 8s wakeup.

#define SLEEPLOOPS 358 // ca *8s pr to get roughly 49min sleep. Relay_254: 358
#define MY_NODE_ID 254 //<============================= Unique NodeID

IPAddress serverIP(xxx, xxx, xxx, xxx); //udp server address
#define REMOTE_PORT 1234   //udp server port
const char PINNUMBER[]     = ""; //The MKRNB lib needs this even if its blank
bool     debugNB           = false;  //show AT commands

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

// initialize the library instance
NBClient client;
GPRS gprs;
NB nbAccess(debugNB);
NBUDP udp;

unsigned long previousMillis; //used for sleep trigger
unsigned int hourCounter = 0;  //total hours since boot
unsigned long packetsCounter = 0; //total packets sent since boot
unsigned long bytesCounter = 0; //total bytes sent since boot
unsigned int packetsLastHourCounter = 0;

byte wdtCounter, wdtCounterReset;
WDTZero MyWatchDoggy; //generates a reset if software loop does not clear WDT on time (generally modem issues).

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
    pinMode(SARA_PWR_ON, OUTPUT);
    // initialize serial communications and wait for port to open:
    //Serial.begin(115200); <started by mysensors
    //while (!Serial);
    delay(500);
    Serial.println("Initial wait 10s to defend against zombies");
    delay(10000); //Blocking delay for zombie protection

    MyWatchDoggy.setup(WDT_SOFTCYCLE2M); //watchdog generates a reset if software loop does not clear WDT within every 2 min

    //power on and reset modem
    digitalWrite(SARA_PWR_ON, HIGH);
    wait(100);
    pinMode(SARA_RESETN, OUTPUT);
    digitalWrite(SARA_RESETN, HIGH);
    wait(100);
    digitalWrite(SARA_RESETN, LOW);

    connectModem();  //init the modem

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
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());
        wait(50); //to not overload modem
        udpSendString(serverIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_0"); //Indicate gateway sleeping

        //Serial.println("DEBUG entering sleep section, shutting down modem etc");
        wait(500); //delay to allow mobile modem to finish sending before turning off.

        nbAccess.secureShutdown(); //Send modem to sleep
        client.flush(); //Cleanup, just in case
        client.stop();  //Cleanup, just in case
        for (int i = 0; i < SLEEPLOOPS; i++) {  //Sleep Loop, 8s ca each.
            digitalWrite(MY_RF24_CE_PIN, LOW);  //DEBUG: force NRF to output/low power mode. Fighting MySensors blobs.
            LowPower.deepSleep();  //wake every 8s by watchdog timer
            MyWatchDoggy.clear(); //Clear watchdog timer, if not done within max 2min or it will reset the cpu.
            digitalWrite(LED_BUILTIN, HIGH); //flash to tell still alive
            wait(50);
            digitalWrite(LED_BUILTIN, LOW);
        }
        //Serial.println("DEBUG end sleep section,");
        //wake up radios
        connectModem();  //wakes up and inits modem
        digitalWrite(MY_RF24_CE_PIN, HIGH); //after modem to avoid potential NRF interrupt mishaps

        sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_1"); //Indicate gateway ready
        hourCounter++;
        packetsLastHourCounter = 0;
        previousMillis = millis();
    }
} //=============END loop=================

//===============functions below=========================
void connectModem() {
    digitalWrite(SARA_PWR_ON, HIGH); //just in case, ensure the thing has power
    digitalWrite(LED_BUILTIN, HIGH); //turn on led to indicate connecting
    Serial.println("Booting modem");
    boolean connected = false;
    while (!connected) {
        if ((nbAccess.begin(PINNUMBER) == NB_READY) && (gprs.attachGPRS() == GPRS_READY)) {
            connected = true;
        } else {
            Serial.println("Connecting...");
            wait(1000);
        }
        udp.begin(5000);
    }
    digitalWrite(LED_BUILTIN, LOW); //turn off to indicate modem OK
    Serial.println("Modem connected");
} //END connectModem

void udpSendString(IPAddress udpServer, unsigned int udpServerport, String message) {  //used from loop to send from msgBuffer stack. returns sent bytes or 0 if failed.
    boolean packetSuccess = 0;
    int sentBytes = 0;
    udp.beginPacket(udpServer, udpServerport);
    sentBytes = udp.print(message); //udp.print nicely converts string to hex before send
    packetSuccess = udp.endPacket(); //0 = failed. NB: might still be successful if first packet after modem 30s+ idle,

    if (packetSuccess == 0) { //packet failed (NB: might still be sent OK if first packet after long idle).
        Serial.print("Send unsure, bytes:"); Serial.println(sentBytes); //verify on server side if was successful, usually are...
        wait(200); //give modem some time to wake up if was first packet in a while
    } else {
        Serial.print("Send OK, bytes:"); Serial.println(sentBytes);
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



