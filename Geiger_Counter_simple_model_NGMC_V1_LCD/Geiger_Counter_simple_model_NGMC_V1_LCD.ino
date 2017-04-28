/*
    GPL ref, based on https://github.com/majek/dump/blob/master/arduino/ard-01/geiger.pde

    HW: Should add 10uF cap between gnd and reset to avoid reset when enabling serial.

    Geiger counter details:
    Geiger interrupt 0 (pin D2 on arduino mini - see attachInterrupt).
    Serial USB output 115200. Indicates CPM, µSv og µSv average (floating average over 10 readings)
     Example output: cpm: 20 uSv/h: 0.16 uSv/h avg: 0.17
    Softserial for bluetooth mobile reporting. Uses pin 11=RX, 12=TX (RX not in use for now). Select app below
     Format required by app:  Evalue1,value2,value3...\n. Data sent are: cpm,usv_average,usv_accumulated
     Using HC-06 bt device, usually pairing passcode is 1234.
    External 5 LED row indicator support with fade for increased resolution. See threshold definitions below.
    LCD support I2C, A4=SDA, A5=SCL. 
     Pin A0 for button (Internal pullup). Events: 
         Short push,  switch LCD backlight on/off.  
         Medium hold: Show battery voltage
         Long hold: Swap piezo (pin 13) on/off. See piezo volume definition below,

    TODO - auto timeout for LCD backlight?

    Misc bloat ideas:.
      TODO: HWmod Store to SD card?
      TODO: HWmod RTC clock for logging

*/
//Serial adaptation mobile app, uncomment one 
#define app_btgraphics //Adapted for app  "Bluetooth Terminal/Graphics" by Emrecan ÇETİN. 
//#define app_ezplotter  //Adapted for ezPlotter by i2A systems

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x3F, 20, 4);
SoftwareSerial btSerial(11, 12); // RX, TX for bluetooth adapter

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000. 
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch
#define BACKLIGHTBUTTON A0 //pushbutton to switch backlight. remember pull down resistor.
#define PIEZOVOLUME 20 //10-80 seems OK range, Controls microeconds pulse duration to piezo. More than 80 probably no effect since pin already are fully on.

#define USV_CONVERSION 123.147092360319  //conversion factor for J305 tube. Factor: 0.00812037037037
//link to data for J305: https://www.cooking-hacks.com/documentation/tutorials/geiger-counter-radiation-sensor-board-arduino-raspberry-pi-tutorial/
//#define USV_CONVERSION 151.5 //for M4011 tube,
//#define USV_CONVERSION 175.43 //for SBM-20 tube

// CPS threshold values for the led bar (resets 10 times pr sec),
/*#define TH0 40
#define TH1 80
#define TH2 120
#define TH3 160
#define TH4 200
*/
//DEBUG VALUES - only have low radiation source.
#define TH0 4
#define TH1 8
#define TH2 12
#define TH3 16
#define TH4 20


//Pins used for LED array. Must be PWM capable
int ledArray [] = {3, 5, 6, 9, 10};

// Variables
unsigned long counts = 0;   //variable for GM Tube events
unsigned long cpm = 0;      //variable for CPM
unsigned long ledcps = 0;      //led counts pr. 10/second
float ledfadecps = 0;      //for smoothing/fadeout
unsigned int multiplier;  //variable for calculation CPM in this sketch
unsigned long previousMillis;  //variable for time measurement
unsigned long previousLedMillis;  //variable for time measurement LED barreset
unsigned long debounceMillis;   // debounce time variable for input buttons
int  ButtonStateCounter = 0;// hold counter.
float usv_average = 0.20; //variable for uSv, starting with avg. 0.20 - roughly normal background radiation.
float usv_average_old = 0.20; //variable for uSv last reading for LCD arrow, starting with avg. 0.20
float usv_accumulated = 0; //variable.for accumulated since boot/reset
boolean lcd_mode = 1; //used to swap LCD info
boolean lcdbacklightstate = 1; //backlight state (on at boot)
boolean piezoenabled = 1; //swap pin 13 (and onboard led) on event. Switched by holding button long.


void setup() {
    multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
    Serial.begin(115200);
    btSerial.begin(9600);
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    attachInterrupt(0, tube_impulse, FALLING);   //define external interrupt 0

    ledFade(TH4 - 1); //all on LEDBAR

    lcd.init();                      // initialize the lcd
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Radiation Sensor");
    lcd.setCursor(2, 1);
    lcd.print("Please wait");

    pinMode(BACKLIGHTBUTTON, INPUT_PULLUP);  //input push button for backlight switching

    //flash onboard led to indicate setup done
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(1500);
    ledFade(TH2 - 1); //ledbar medium
    digitalWrite(13, LOW);
    delay(1000);

    ledFade(0); //zero ledbar

    Serial.println("Arduino setup complete");
}

void loop() {
    unsigned long currentMillis = millis();

    //LCD, bluetooth and serial output/updates section
    if (currentMillis - previousMillis > LOG_PERIOD) {
        previousMillis = currentMillis;
        cpm = counts * multiplier; //adjustable by constants

        //const float conversion_factor
        float usv = (float)cpm / USV_CONVERSION;
        usv_average = ((usv_average * 9 + usv) / 10);
        usv_accumulated = usv_accumulated + (usv / (MAX_PERIOD / LOG_PERIOD)) / 60; //accumulated since boot/reset

        lcd.clear();
        lcd.setCursor(0, 0);
        if (lcd_mode) { //swaps between two LCD infos on first line
            if (cpm < 10) lcd.print(" "); //add spaces to avoid jumping on 1, 10 digit numbers
            if (cpm < 100) lcd.print(" ");
            if (cpm < 1000) lcd.print(" ");
            lcd.print(cpm);
            lcd.print("cpm");
            lcd.setCursor(8, 0);
            lcd.print(usv, 2);
            lcd.print((char)228); //special char µ
            lcd.print("Sv");
            lcd_mode = 0;
        } else {
            lcd.print(" ");
            lcd.print((char)228); //special char µ
            lcd.print("Sv acc: ");
            lcd.print(usv_accumulated);
            lcd_mode = 1;
        }
        lcd.setCursor(0, 1); //second line
        if (usv_average > usv_average_old) {
            lcd.print((char)126); //special char, right arrow (rom missing up)
        } else if (usv_average < usv_average_old) {
            lcd.print((char)127); //special char, left arrow (rom missing down)
        }  else {
            lcd.print(" "); //no change, space only
        }
        lcd.print(usv_average, 3);
        lcd.print((char)228); //special char µ
        lcd.print("Sv/h avg");
        usv_average_old = usv_average; //update with last reading

        //USB serial
        Serial.print("cpm: ");
        if (cpm < 10) Serial.print(" "); //adds extra space if single digit to clean up formatting
        //if (cpm < 100) Serial.print(" "); //adds extra space if single digit to clean up formatting
        //if (cpm < 1000) Serial.print(" "); //adds extra space if single digit to clean up formatting
        Serial.print(cpm);
        Serial.print(" uSv/h: ");
        Serial.print(usv);
        Serial.print(" uSv/h avg: ");
        Serial.print(usv_average);
        Serial.print(" uSv/h acc: ");
        Serial.println(usv_accumulated, 4);

        //Bluetooth softserial, adapted for app
#ifdef app_btgraphics
        btSerial.print("E");
        btSerial.print(cpm);
        btSerial.print(",");
        btSerial.print(usv_average);
        btSerial.print(",");
        btSerial.print(usv_accumulated);
        btSerial.print("\n");
#endif
#ifdef app_ezplotter
        btSerial.print(cpm);
        btSerial.print(",");
        btSerial.print(usv_average);
        btSerial.print(",");
        btSerial.println(usv_accumulated);
#endif

        counts = 0;
    } //END LCD, bluetooth and serial output/updates section

    //Ledbar section
    if (currentMillis - previousLedMillis > 200) { //run x times pr second
        previousLedMillis = currentMillis;
        //calculate fading effect. Not super accurate to get an "analog feel" perhaps
        if (ledcps >= ledfadecps) ledfadecps=ledcps; else ledfadecps=ledfadecps/1.5+ledcps; 
        ledFade(ledfadecps); 
        ledcps = 0; //reset counter
    } //END Ledbar section

    //Button event section
    if (currentMillis - debounceMillis > 500 || (!digitalRead(BACKLIGHTBUTTON) && !ButtonStateCounter)) {  //check interval. Or check here to catch early press
        debounceMillis = currentMillis; // reset counter
        if (!digitalRead(BACKLIGHTBUTTON)) ButtonStateCounter++; // Button is held down, increase button counter

        //button release events. Checks counter to do stuff
        if (digitalRead(BACKLIGHTBUTTON) && ButtonStateCounter == 1) { //Button state counter larger than 0, and button is released.
            if (lcdbacklightstate) {
                lcd.noBacklight(); // turn off backlight
                lcdbacklightstate = 0; debounceMillis;
            } else {
                lcd.backlight(); // turn on backlight.
                lcdbacklightstate = 1;
            }
        }
        //below here will cycle forwards while button is held down. 
        if (ButtonStateCounter == 2)  {  //long press 1. Just do show battery level. Do not reset counter here, done below when button released.
            float vccread = readVcc();
            vccread = vccread / 1000; //convert from mv to V
            lcd.backlight(); // turn on backlight in case it is off
            lcdbacklightstate = 1;
            lcd.clear();
            lcd.setCursor(0, 0); //first line
            lcd.print("Battery Voltage:");
            lcd.setCursor(2, 1); //second line
            lcd.print(vccread);
            lcd.print("V ");
            ButtonStateCounter++; //increased one to avoid risk being done twice when key released
        }
        if (ButtonStateCounter == 5)  {  //long press 2. Swap piezoenabled booleand and inform on LCD
            piezoenabled=!piezoenabled;
            lcd.backlight(); // turn on backlight in case it is off
            lcdbacklightstate = 1;
            lcd.clear();
            lcd.setCursor(0, 0); //first line
            lcd.print("Piezo buzzer:");
            lcd.setCursor(2, 1); //second line
            if (piezoenabled) lcd.print("ON"); else lcd.print("OFF");
            ButtonStateCounter++; //increased one to avoid risk being done twice when key released
        }
        //Might add more stuff here for event 4, 6, 8 etc to count through every 2. sec..
        if (digitalRead(BACKLIGHTBUTTON)) ButtonStateCounter = 0; // Cleanup and reset button counter. Events handled above.
    } //END Button event section
} //END loop

//-----------------------------Functions/events below here----------------------------
void tube_impulse() {  //function for capturing interrupt events from Geiger Kit
    counts++;
    ledcps++;
    //switch state onboard led to indicate response
    //digitalWrite(13, !digitalRead(13));
    
    //"ping" pin 13 if piezoenabled. 
     if (piezoenabled) {
       digitalWrite(13, HIGH);
       delayMicroseconds(PIEZOVOLUME); //Not ideal with blocking in here but some delay needed for audible. Longer duration=more volume)
       digitalWrite(13, LOW);
     }
}

void ledFade(int value) { //function for 5x LED row update with fading effect for increased resolution.
    if (value < TH0) {
        analogWrite(ledArray[0], map(value, 0, TH0, 0, 255));
        digitalWrite(ledArray[1], LOW); digitalWrite(ledArray[2], LOW); digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if (value < TH1) {
        digitalWrite(ledArray[0], HIGH);
        value = - TH0; //subtract previous threshold
        analogWrite(ledArray[1], map(value, 0, TH1, 0, 255));
        digitalWrite(ledArray[2], LOW); digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if (value < TH2) {
        digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH);
        value = - TH1; //subtract previous threshold
        analogWrite(ledArray[2], map(value, 0, TH2, 0, 255));
        digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if (value < TH3) {
        digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH);
        value = - TH2; //subtract previous threshold
        analogWrite(ledArray[3], map(value, 0, TH3, 0, 255));
        digitalWrite(ledArray[4], LOW);
    } else if (value < TH4) {
        digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH); digitalWrite(ledArray[3], HIGH);
        value = - TH3; //subtract previous threshold
        analogWrite(ledArray[4], map(value, 0, TH4, 0, 255));
    } else {
        digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH); digitalWrite(ledArray[3], HIGH); digitalWrite(ledArray[4], HIGH);
    }
}

long readVcc() {
    // Read 1.1V reference against AVcc
    // set the reference to Vcc and the measurement to the internal 1.1V reference
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
#else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif

    delay(2); // Wait for Vref to settle
    ADCSRA |= _BV(ADSC); // Start conversion
    while (bit_is_set(ADCSRA, ADSC)); // measuring

    uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
    uint8_t high = ADCH; // unlocks both

    long result = (high << 8) | low;

    result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
    return result; // Vcc in millivolts
}


