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
     Power controlled via transistor on pin BTTRANSISTOR (to turn fully off).
    External 5 LED row indicator support with fade for increased resolution. See threshold definitions below.
    LCD support I2C, A4=SDA, A5=SCL.
    Pin A0 for button (Internal pullup). Events:
         Short push,  switch LCD backlight on/off.
         Hold button: Cycle through: VCC voltage, piezo on/off, Piezo volume, Bluetooth, (end/no change). Release at piezo/bluetooth to change state/volume.
    Persistent values stored in EEPROM:
      Piezo state (adress 0)
      Piezo volume (adress 1)
      Bluetooth power-pin on/off (adress 2)
      Backlight auto power off (adress 3)

    TODO - Move uSv accumulated to menu (first/before battery level)
    TODO: replace uSv acc with highest CPM peak last 10 reads.

    Misc bloat ideas:.
      TODO: HWmod Store to SD card?
      TODO: HWmod RTC clock for better logging

*/
//Serial adaptation mobile app, uncomment one
#define app_btgraphics //Adapted for app  "Bluetooth Terminal/Graphics" by Emrecan ÇETİN. 
//#define app_ezplotter  //Adapted for ezPlotter by i2A systems

#include <Wire.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x3F, 20, 4);
SoftwareSerial btSerial(11, 12); // RX, TX for bluetooth adapter

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000. 
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch
#define CPMSAMPLES 20 //Used to store CPM over LOG_PERIOD for peak display. Max 255 (using byte)
#define BACKLIGHTAUTOOFF 4  //Max 255. If enabled, auto off timer. Number of LOG_PERIOD cycles before turning off (no ned for separate millis check here).
#define PUSHBUTTON A0 //pushbutton to switch backlight. remember pull down resistor.
#define PIEZOPIN 7 //Pin for piezo "pings"
#define BTTRANSISTOR 4 //Pin for bluetooth enable (I've used NPN transitor for gnd enable)

#define USV_CONVERSION 123.147092360319  //conversion factor for J305 tube. Factor: 0.00812037037037
//link to data for J305: https://www.cooking-hacks.com/documentation/tutorials/geiger-counter-radiation-sensor-board-arduino-raspberry-pi-tutorial/
//#define USV_CONVERSION 151.5 //for M4011 tube,
//#define USV_CONVERSION 175.43 //for SBM-20 tube

//Piezo volume levels. 10-80 seems OK range, Controls microeconds pulse duration to piezo. More than 80 probably no effect since pin already are fully on.
#define PIEZOVOLUME1 30//5
#define PIEZOVOLUME2 60 //10
#define PIEZOVOLUME3 80 //20
#define PIEZOVOLUME4 100 //30

// CPS threshold values for the led bar (resets 10 times pr sec),
/*  #define TH0 40
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
unsigned int cpmPeakTable[CPMSAMPLES];      //variable for CPM peak store
byte cpmPeakTableCounter=0;      //variable for CPM peak store
unsigned int cpmPeak=0;      //used for result from CPM peak store
unsigned long ledcps = 0;      //led counts pr. 10/second
float ledfadecps = 0;      //for smoothing/fadeout
unsigned int multiplier;  //variable for calculation CPM in this sketch
unsigned long previousMillis;  //variable for time measurement
unsigned long previousLedMillis;  //variable for time measurement LED barreset
unsigned long debounceMillis;   // debounce time variable for input buttons
int  ButtonStateCounter = 0;// hold counter.
float usv_average = 0; //variable for uSv,
float usv_average_old = 0; //variable for uSv last reading for LCD arrow, 0
float usv_accumulated = 0; //variable.for accumulated since boot/reset
boolean lcd_mode = 1; //used to swap LCD info; TODO: Make this EEPROM value
boolean lcdbacklightstate = 1; //backlight state (on at boot).
boolean piezoenabled; //state of clcker, read from EEPROM adress 0
byte piezovolume;//state of clcker, read from EEPROM adress 1
boolean bluetoothenabled; //bluetooth enabled, EEPROM adress 2
boolean backlightautooff; //backlight auto power off. EEPROm adress 4
byte backlightautooffcounter = 0; //counter for backlight checks

void setup() {
    pinMode(PIEZOPIN, OUTPUT); //Piezo pin
    pinMode(BTTRANSISTOR, OUTPUT); //Bluetooth power pin
    //read values from EEPROM:
    piezoenabled = EEPROM.read(0); //EEPROM adress 0
    piezovolume = EEPROM.read(1); //EEPROM adress 1
    bluetoothenabled = EEPROM.read(2); //EEPROM adress 2
    backlightautooff = EEPROM.read(3); //EEPROM adress 3
    digitalWrite(PIEZOPIN, piezoenabled);
    digitalWrite(BTTRANSISTOR, bluetoothenabled);
    pinMode(13, OUTPUT); //Onboard LED
    digitalWrite(13, LOW);
    pinMode(PUSHBUTTON, INPUT_PULLUP);  //input push button for backlight switching
    attachInterrupt(0, tube_impulse, FALLING);   //define external interrupt 0

    multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
    Serial.begin(115200);
    btSerial.begin(9600);
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

    ledFade(TH4 - 1); //all on LEDBAR

    lcd.init();                    
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Radiation Sensor");
    lcd.setCursor(2, 1);
    lcd.print("Please wait");

    //flash ledrange
    delay(100);
    ledFade(TH3 - 1); //ledbar medium
    delay(100);
    ledFade(TH2 - 1); //ledbar medium
    delay(100);
    ledFade(TH1 - 1); //ledbar medium
    digitalWrite(13, HIGH); //flash onboard led
    delay(100);
    ledFade(0); //zero ledbar
    digitalWrite(13, LOW);

    Serial.println("Arduino setup complete");
}

void loop() {
    unsigned long currentMillis = millis();

    //LCD, bluetooth and serial output/updates section
    if (currentMillis - previousMillis > LOG_PERIOD) {
        previousMillis = currentMillis;
        cpm = counts * multiplier; //adjustable by constants

        //CPM peak store and find
        //add current value to CPM table
        if (cpmPeakTableCounter < CPMSAMPLES)  cpmPeakTableCounter++; else cpmPeakTableCounter=0; //increase or reset counter
        cpmPeakTable[cpmPeakTableCounter] = cpm;      //add cpm to CPM peak store
        
        //reset peak value and find highest
        cpmPeak=0;
        int i;
        for (i = 0; i < CPMSAMPLES; i = i + 1) {
          if (cpmPeakTable[i] > cpmPeak) cpmPeak = cpmPeakTable[i];
        }
        //DEBUG----------------------------------------------
        /*Serial.print("DEBUG cpmpeak:");
        Serial.print(cpmPeak);
        Serial.print(" ");*/
        //DEBUG---------------------------------------------- 
        //const float conversion_factor
        float usv = (float)cpm / USV_CONVERSION;
        usv_average = ((usv_average * 9 + usv) / 10);
        usv_accumulated = usv_accumulated + (usv / (MAX_PERIOD / LOG_PERIOD)) / 60; //accumulated since boot/reset

        if (digitalRead(PUSHBUTTON)) { //LCD update section. Check if button down to not interfere with menu
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
                lcd.print("CPM Peak: ");
                lcd.print(cpmPeak);
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
        } //END LCD update section

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

        //check if backlight auto off is enabled. If true update timer and change value.
        if (backlightautooff && lcdbacklightstate && digitalRead(PUSHBUTTON)) { //dont if holding down pushbutton..
            if (backlightautooffcounter < BACKLIGHTAUTOOFF) backlightautooffcounter++;
            else { //reached counter value. Turn off backlight
                lcd.noBacklight(); // turn off backlight
                lcdbacklightstate = 0;
            }
        } 
        counts = 0;
    } //END LCD, bluetooth and serial output/updates section

    //Ledbar section
    if (currentMillis - previousLedMillis > 200) { //run x times pr second
        previousLedMillis = currentMillis;
        //calculate fading effect. Not super accurate to get an "analog feel" perhaps
        if (ledcps >= ledfadecps) ledfadecps = ledcps; else ledfadecps = ledfadecps / 1.5 + ledcps;
        ledFade(ledfadecps);
        ledcps = 0; //reset counter
    } //END Ledbar section

    //Button check and LCD/parameter update section. On millis interval.
    if (currentMillis - debounceMillis > 500) {  //check interval. || check here is to catch early press.
        if (!digitalRead(PUSHBUTTON)) { // Button is held down, increase button counter and turn on backlight if off
            ButtonStateCounter++;
            debounceMillis = currentMillis; // reset counter
        }
        //button hold and release events. Checks counter and button state to do stuff
        if (ButtonStateCounter > 0) {
            if (ButtonStateCounter == 1) { //Just short press. Switch baclkight.
                if (digitalRead(PUSHBUTTON)) { //button released while here
                    if (lcdbacklightstate) {
                        lcd.noBacklight(); // turn off backlight
                        lcdbacklightstate = 0;
                    } else {
                        lcd.backlight(); // turn on backlight.
                        lcdbacklightstate = 1;
                        backlightautooffcounter = 0; //reset backlight auto off counter
                    }
                    ButtonStateCounter = 0;
                }
            } else if (ButtonStateCounter >= 1 && ButtonStateCounter <= 3) {  //long press events from here. Range to get a reasonable delay. Backlight turned on always from here on out.
                //display accumulated
                lcd.backlight();
                lcdbacklightstate = 1;
                lcd.clear();
                lcd.setCursor(0, 0); //first line
                lcd.print("Dose since boot:");
                lcd.setCursor(2, 1); //second line
                lcd.print(usv_accumulated);
                lcd.print((char)228); //special char µ
                lcd.print("Sv");
                if (digitalRead(PUSHBUTTON)) ButtonStateCounter = 0; //button released while here                
            } else if (ButtonStateCounter >= 4 && ButtonStateCounter <= 6) {  //long press events from here. Range to get a reasonable delay. Backlight turned on always from here on out.
                float vccread = readVcc();
                vccread = vccread / 1000; //convert from mv to V
                lcd.backlight();
                lcdbacklightstate = 1;
                lcd.clear();
                lcd.setCursor(0, 0); //first line
                lcd.print("Battery Voltage:");
                lcd.setCursor(2, 1); //second line
                lcd.print(vccread);
                lcd.print("V ");
                if (digitalRead(PUSHBUTTON)) ButtonStateCounter = 0; //button released while here
            } else if (ButtonStateCounter >= 7 && ButtonStateCounter <= 9) { //Range here to get a reasonable delay. Switch piezo section
                if (!digitalRead(PUSHBUTTON))  {
                    lcd.clear();
                    lcd.setCursor(0, 0); //first line
                    lcd.print("Piezo clicker:");
                    lcd.setCursor(2, 1); //second line
                    if (piezoenabled) lcd.print("ON "); else lcd.print("OFF");
                } else { //button released while here, change value, update EEPROM
                    piezoenabled = !piezoenabled;
                    eepromWrite(0, piezoenabled); //store value in eerpom
                    lcd.setCursor(2, 1); //second line
                    if (piezoenabled) lcd.print("ON "); else lcd.print("OFF");
                    ButtonStateCounter = 0;
                }
            } else if (ButtonStateCounter >= 10 && ButtonStateCounter <= 12) { //Range here to get a reasonable delay. Switch piezo section
                if (!digitalRead(PUSHBUTTON))  {
                    lcd.clear();
                    lcd.setCursor(0, 0); //first line
                    lcd.print("Piezo volume:");
                    lcd.setCursor(2, 1); //second line
                    lcd.print(piezovolume);
                } else { //button released while here, change value, update EEPROM
                    if (piezovolume <= PIEZOVOLUME1) piezovolume = PIEZOVOLUME2; //smaller than to handle empty EEPROM value.
                    else if (piezovolume == PIEZOVOLUME2) piezovolume = PIEZOVOLUME3;
                    else if (piezovolume == PIEZOVOLUME3) piezovolume = PIEZOVOLUME4;
                    else if (piezovolume >= PIEZOVOLUME4) piezovolume = PIEZOVOLUME1; //go back to 10, larger than if bugs..
                    eepromWrite(1, piezovolume); //store value in eerpom
                    lcd.setCursor(2, 1); //second line
                    lcd.print(piezovolume);
                    lcd.print("  ");//to clear out old values if going down to single digit
                    ButtonStateCounter = 0;
                }

            } else if (ButtonStateCounter >= 13 && ButtonStateCounter <= 15) { //Range here to get a reasonable delay. Switch bluetooth section
                if (!digitalRead(PUSHBUTTON))  {  //button is held down, just update display
                    lcd.clear();
                    lcd.setCursor(0, 0); //first line
                    lcd.print("Bluetooth:");
                    lcd.setCursor(2, 1); //second line
                    if (bluetoothenabled) lcd.print("Enabled "); else lcd.print("Disabled");
                } else { //button released while here, change value, update EEPROM
                    bluetoothenabled = !bluetoothenabled;
                    eepromWrite(2, bluetoothenabled); //store value in eerpom
                    digitalWrite(BTTRANSISTOR, bluetoothenabled); //switch output pin
                    lcd.setCursor(2, 1); //second line
                    if (bluetoothenabled) lcd.print("Enabled "); else lcd.print("Disabled");
                    ButtonStateCounter = 0;
                }
            } else if (ButtonStateCounter >= 16 && ButtonStateCounter <= 18) { //Range here to get a reasonable delay. Switch bluetooth section
                if (!digitalRead(PUSHBUTTON))  {  //button is held down, just update display
                    lcd.clear();
                    lcd.setCursor(0, 0); //first line
                    lcd.print("LCDlight autooff:");
                    lcd.setCursor(2, 1); //second line
                    if (backlightautooff) lcd.print("Enabled "); else lcd.print("Disabled");
                } else { //button released while here, change value, update EEPROM
                    backlightautooff = !backlightautooff;
                    eepromWrite(3, backlightautooff); //store value in eerpom
                    lcd.setCursor(2, 1); //second line
                    if (backlightautooff) lcd.print("Enabled "); else lcd.print("Disabled");
                    backlightautooffcounter = 0;
                    ButtonStateCounter = 0;
                }
            } else if (ButtonStateCounter > 19)  {  //catch all, indicate end of menu/no change. adjust value according to above choices
                if (!digitalRead(PUSHBUTTON))  {  //button is held down, just update display
                    lcd.clear();
                    lcd.setCursor(0, 0); //first line
                    lcd.print("End of menu");
                    lcd.setCursor(0, 1); //second line
                    lcd.print("Release button");
                } else { //button released while here, indicate by changing second line
                    lcd.setCursor(0, 1); //second line3
                    lcd.print("Awaiting update ");
                    ButtonStateCounter = 0;
                }
            } //end long if/else chain
        } //end if ButtonStateCounter > 0
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
        digitalWrite(PIEZOPIN, HIGH);
        delayMicroseconds(piezovolume); //Not ideal with blocking in here but some delay needed to be audible. (Longer duration=more volume)
        digitalWrite(PIEZOPIN, LOW);
    }
}

void eepromWrite(byte addr, byte value) { //function to check if diff value/update needed and disable interrupts while writing to EEPROM,
    while (!eeprom_is_ready());
    if (EEPROM.read(addr) != value) { //new value, update EEPROM
        cli();
        EEPROM.write(addr, value);
        sei();
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


