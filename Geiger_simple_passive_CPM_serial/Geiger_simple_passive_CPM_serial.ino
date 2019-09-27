//Simple script to push CPM data to listener. Usv calculations etc on receiver.
//pin 8 for piezo output
//pin2 interrupt from geiger.
//TODO: BLE support?

#include <SPI.h>
#define LOG_PERIOD 30000     //Logging period in milliseconds, recommended value 15000-60000.
#define MAX_PERIOD 60000    //Maximum logging period
#define CPMTHRESHOLD 50

unsigned long counts;             //variable for GM Tube events
unsigned long cpm;                 //variable for CPM
unsigned int multiplier;             //variable for calculation CPM in this sketch
unsigned long previousMillis;      //variable for time measurement
boolean enableclicker=0; //enable piezo if above CPMTHRESHOLD. Start enabled for debug

void setup() {                                              //setup procedure
    counts = 0;
    cpm = 0;
    multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
    PORTB = PORTB & 0xFE;  //pin8 low
    Serial.begin(115200);                                    // start serial monitor

    // uncommennt if you have time-out problem to connect with Radiation Logger
    //  delay(2000);
    //  Serial.write('0');                                      // sending zero to avoid connection time out with radiation logger
    //  delay(2000);
    //  Serial.write('0');                                     // sending zero to avoid connection time out with radiation logger

    pinMode(2, INPUT);                                   // set pin INT0 input for capturing GM Tube events
    digitalWrite(2, HIGH);                                 // turn on internal pullup resistors, solder C-INT on the PCB
    attachInterrupt(0, tube_impulse, FALLING);  //define external interrupts

    //Serial.println("Startup");                              // Startup indicator disabled to not confuse scripts..

    //some ticks to indicate startup:
    for (int i=0; i <= 50; i++) {
      PORTB = PORTB | 0x01; //pin8 high
      delayMicroseconds(10); 
      PORTB = PORTB & 0xFE;  //pin8 low
      delay(100);
    }
}

void loop() {                                              //main cycle
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis > LOG_PERIOD) {
        previousMillis = currentMillis;
        cpm = counts * multiplier;
        if (cpm < CPMTHRESHOLD) {enableclicker=0;} //ensure clicker is disabled when below threshold 
        Serial.print(cpm);                              // send cpm data to Radiation Logger
        Serial.write(' ');                                // send null character to separate next data
        //DEBUG
        //Serial.print(" clickerstate: ");                            
        //Serial.println(enableclicker);                               
        counts = 0;
    }
}

void tube_impulse() {              //procedure for capturing events from Geiger Kit
    counts++;

    if (counts *multiplier > CPMTHRESHOLD or enableclicker) {
      enableclicker=1; //only disabled in main loop
      PORTB = PORTB | 0x01; //pin8 high
      delayMicroseconds(10); 
      PORTB = PORTB & 0xFE;  //pin8 low
    }
}
