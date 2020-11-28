/***********************************************************************
    Board: Leonardo pro micro 3.3v, 8Mhz internal Atmega32u4
    Ref: https://forum.arduino.cc/index.php?topic=409415.0

    PINS Used:
    NBIOT RX/TX: 8/7 (NB as marked  on the radio board).
    NRF24:
    18/A0 CE
    10 CSN/CS
    15 SCK  <- NB: also led builtin. TX flashes anyway so desolder..
    16  MOSI
    14  MISO
    2  IRQ

    Voltage divider lipo/raw ---R1 47k--- PinA3 ---R2 22k--- gnd. 0.1uF cap on PinA3 to gnd.

    Note: Only reads mysensors float values. Incoming NRF packets are stored via interrupt in a message stack awaiting transmit loop pickup.
    Listens for 11 minutes, then sleeps 49 (sensor nodes runs at a 10min transmit interval)

    Changes v6.3 mostly server-side. New dest. IP and APN to telenor.iot. Also adding MY_NODE_ID@ as a prefix for all packets  ...
    which breaks with general mysensors protocol but allows for separation server side.
***********************************************************************/
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>

#define SLEEPLOOPS 341 //*8s pr to get roughly 49min sleep. Adjusted clockdrift NodeID 251: 326, Nodeid 252: 337, NodeID 253: 341
#define MY_NODE_ID 253 //<============================= Unique NodeID

#define POWERPIN A3 //to read battery voltage 
//#define MY_DEBUG // Enable debug prints to serial monitor
#define MY_BAUD_RATE 38400
#define MY_RADIO_RF24
#define MY_RF24_PA_LEVEL RF24_PA_LOW
#define MY_RF24_CE_PIN 18 //A0 on board
#define MY_RF24_CS_PIN 10 //CSN
//#define MY_RF24_IRQ_PIN 2
//#define MY_RX_MESSAGE_BUFFER_FEATURE //use IRQ
//#define MY_RX_MESSAGE_BUFFER_SIZE (3)
#define MY_GATEWAY_SERIAL //to force simple gateway mode, workaround for mobile forward
#define MY_DISABLED_SERIAL  //to skip serial wait. 
//#define MY_DISABLE_RAM_ROUTING_TABLE_FEATURE
#include <MySensors.h>

#include <Udp.h>
#include <TelenorNBIoT.h>
#include <SoftwareSerial.h>
SoftwareSerial ublox(8, 7); //RX, TX - as marked on module (it's backwards there..)
TelenorNBIoT nbiot("telenor.iot", 242, 01);

// The remote IP address to send data packets to
// u-blox SARA N2 does not support DNS
IPAddress remoteIP(173, 249, 2, 55);
#define REMOTE_PORT 1234

//Millis used for hourly "cron" job
unsigned long previousMillis;
//unsigned long previousMillisSend;
//unsigned long currentMillis = 0;;
unsigned int hourCounter = 0;
unsigned int packetsCounter = 0;
unsigned int packetsLastHourCounter = 0;
unsigned int loopCounter = 0; //for reset trigger if all goes to hell..

//messagestack
String msgBuffer0 = "";
String msgBuffer1 = "";
String msgBuffer2 = "";
String msgBuffer3 = "";
String msgBuffer4 = "";
String msgBuffer5 = "";
String msgBuffer6 = "";


void setup() {

    pinMode(POWERPIN, INPUT);
    //analogReference(INTERNAL); //1.1V
    delay(5000);
    //Serial.begin(38400);
    ublox.begin(9600);
    // Try to initialize the NB-IoT module until it succeeds
    //Serial.print("Connecting to NB-IoT module...\n");
    while (!nbiot.begin(ublox)) {
        //Serial.println("Begin failed. Retrying...");
        delay(1000);
    }
    /*  Serial.print("IMSI: ");
        //Serial.println(nbiot.imsi());
        Serial.print("IMEI: ");
        //Serial.println(nbiot.imei());
    */
    // Try to create a socket until it succeeds
     while (!nbiot.createSocket()) {
         delay(100);
     }
    nbiot.isConnected(); //just do check/read to force serial sync.

    //Power saving. NBNBNB!!! disable if need MY_DEBUG and serial debug further down
    //Serial.end();
    power_adc_disable(); //just to mess about - enabled again in readVoltage(). DEBUG can remove.
    power_usart0_disable();// Serial (USART)
    power_usart1_disable();// Serial (USART)
    power_twi_disable(); // TWI (I2C)
    power_usb_disable();
    USBCON |= (1 << FRZCLK);             // Freeze the USB Clock
    PLLCSR &= ~(1 << PLLE);              // Disable the USB Clock (PPL)
    USBCON &=  ~(1 << USBE  );           // Disable the USB

    /*** Setup the WDT ***/
    MCUSR &= ~(1 << WDRF); /* Clear the reset flag. */
    /*  In order to change WDE or the prescaler, we need to
        set WDCE (This will allow updates for 4 clock cycles).
    */
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    WDTCSR = 1 << WDP0 | 1 << WDP3; /* 8.0 seconds */ /* set new watchdog timeout prescaler value */
    WDTCSR |= _BV(WDIE); /* Enable the WD interrupt (note no reset). */

    readVoltage(); //Force 1 read since first on boot seems to fail/show max. Fix OK.
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/11_NBIoT-SaraD2"); //Sketch name
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/12_6.3"); //Sketch version
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_1"); //Indicate gateway ready
    sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());

    previousMillis = millis();
}

void loop() {
    //Send from buffer handler
    if (nbiot.isConnected()) {
        loopCounter=0; 
        if (msgBuffer0 != "") {
            //Serial.println("s0");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer0);
            msgBuffer0 = ""; //clear
        } else if (msgBuffer1 != "") {
            //Serial.println("s1");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer1);
            msgBuffer1 = ""; //clear
        } else if (msgBuffer2 != "") {
            //Serial.println("s2");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer2);
            msgBuffer2 = ""; //clear
        } else if (msgBuffer3 != "") {
            //Serial.println("s3");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer3);
            msgBuffer3 = ""; //clear
        } else if (msgBuffer4 != "") {
            //Serial.println("s4");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer4);
            msgBuffer4 = ""; //clear
        } else if (msgBuffer5 != "") {
            //Serial.println("s5");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer5);
            msgBuffer5 = ""; //clear
        } else if (msgBuffer6 != "") {
            //Serial.println("s6");
            nbiot.sendString(remoteIP, REMOTE_PORT, msgBuffer6);
            msgBuffer6 = ""; //clear
        } else {
            smartSleep(2, CHANGE, 1000); //interrupt pin, interrupt type, delay in ms. In practice heartbeat towards NBIOT. Wake if nrf get packet
        }
    }
    else { //fallback, check stuff, worst case reboot and hope for the best...
        //Serial.println("NC");
        //delay(5000);
        loopCounter++;
        if (loopCounter > 200) reboot(); //watchdog, reboot if never connects. TODO - needs field verification it works in practice or are redundant.
        smartSleep(2, CHANGE, 5000); //interrupt pin, interrupt type, delay in ms
    }

    //Sleep section
    if (millis() - previousMillis > 660000) { //run every 11 min, holds in sleep for 49
        digitalWrite(MY_RF24_CE_PIN, LOW);  //set NRF to output/low power mode
        //power_spi_disable();
        //Serial.println("DEBUG: Entering millis" );
        //Send bye messages directly/out of buffer.
        nbiot.sendString(remoteIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/24_" + String(hourCounter));
        nbiot.sendString(remoteIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/25_" + String(packetsCounter));
        nbiot.sendString(remoteIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/26_" + String(packetsLastHourCounter));
        nbiot.sendString(remoteIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/1/0/38_" + readVoltage());
        nbiot.sendString(remoteIP, REMOTE_PORT, String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_0"); //Indicate gateway sleeping

        //sleep section from here.
        for (int i = 0; i < SLEEPLOOPS; i++) {  //Sleep Loop, 8s ca each. 
            digitalWrite(MY_RF24_CE_PIN, LOW);  //DEBUG: force NRF to output/low power mode. Fighting MySensors blobs.
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);   /* EDIT: could in theory also use SLEEP_MODE_PWR_DOWN for lowest power consumption. */
            sleep_enable();
            sleep_mode();  /* Now enter sleep mode. */
            //sleep_disable();   //not needed?
        }

        digitalWrite(MY_RF24_CE_PIN, HIGH);
        sendMessage(String(MY_NODE_ID)  + "@" + String(MY_NODE_ID) + "/255/3/0/14_1"); //Indicate gateway ready
        hourCounter++;
        packetsLastHourCounter = 0;
        previousMillis = millis();
    }
} //END loop

//------------------------------UTILS BELOW-------------------------------
//ISR(WDT_vect) <- in mysensors blobs
//{
//do nothing, handler for sleep
//}
void reboot() { //reboot, used as a last resort in loop if connect fails constantly...
  wdt_disable();
  wdt_enable(WDTO_15MS);
  while (1) {}
}

void receive(const MyMessage &message)  {//Kicked off by mysensors lib by interrupt apparently. Unpacks to packet array and forwards to sendMessage() function
    String msgValue = String(message.getFloat());
    //  if (!msgValue) {msgValue=String(message.getLong());}

    String messageConstruct = String(MY_NODE_ID)  + "@" + String(message.sender) + "/" + String(message.sensor) +  "/" + String(message.getCommand()) +  "/" + String(message.isAck()) +  "/" + String(message.type) +  "_" + msgValue;
    //Serial.println("messageConstruct: " + messageConstruct);
    packetsCounter++;
    packetsLastHourCounter++;
    sendMessage(messageConstruct);
} //END mysensors message receive events handler

void sendMessage(String payload) { //only adds to send buffer, real IoT sending is done in loop millis handler
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
    } else {
        //Serial.println("BUFFER FULL");
    }
} //END sendMessage

String readVoltage() {
    power_adc_enable(); //ensure thing is up and running.
    delay(300); //stabilize
    float reading = analogRead(POWERPIN);
    power_adc_disable();
    reading = (reading / 1024) * 3.27; //3.28 Convert to actual voltage vs. reference. 
    //Divider lipo-R1(48k)--R2(22k) -gnd. 5V=1.594V
    //denominator = 22000 / (48000 + 22000) = 0.314286.
    reading = reading / 0.314286; //divide by precalculated denominator for voltage divider
    return String(reading, 2);
}



