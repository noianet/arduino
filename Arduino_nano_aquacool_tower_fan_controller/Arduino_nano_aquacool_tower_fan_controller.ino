/*
    REMEMBER: Set nano atmega328 old bootloader before flash <--------------
 
    Temperature driven fan controller, 3 wire. 
    Using unknown thermistors from tower (10k), Vdivider with 10K resistor. Air only reporting, water for fan control.
    Fans run by PWM feeding a basic NPN transistor. MCU PWM frequency increased to avoid audible noise.
    Fan tacho read via interrupt, used for logging and stall detection. Since fans are in parallell and fead via PWM output are a bit rough but good enough.
    
    Serial output example:
    Ta: 20.08 Tw: 21.64 PWMc: 120 PWMa: 150 Fs: 1 RPM: 198
    Ta= Temp air
    Tw = Temp water
    PWMc=current calculated "want" PWM based on temp reading.
    PWMa=averaged from earlier read for hysterisis. Are used to drive PWM signal.
    Fr: internal fanRunningState flag to handle "kick" for startup. 1=fan on. LED_BIULTIN also used led on=fan on. 
    RPM: Pulses from hallsensors (interrupt driven) 
    
    Serial input allows sets manual PWM (1-255). Overrides automatics, set 0 to back to automatic.
    
    Millis logicsteps autodriver runs every 2 sec. Millis Serial reports stats via serial every 10 sec, 

    TODO: Consider only RPM check not zero since seems consistent for extra kick. Remove out fanRunningState completely?
*/
#include <math.h>

#define MAXTEMP 29.0 //Fan on full speed. 
#define MINTEMP  24.0 //FAN off. 
#define MINSPEED 120 //Minimum fan speed, Referenced PWM range 0-255. Needs kick...
#define MINRPM 60 //Minimum fan speed RPM read before doing a kick. 
#define KICKSPEED 255 //Minimum fan startup value. 220 too low/failed once, just start at full blast to be sure...
#define TEMPPINAIR A1
#define TEMPPINWATER A2
#define TACHOPIN 2 //fan hallsensor interrupt (yellow wire on fan)
#define PWMPIN 11 //fan speed

double airTemp = 22; //sensible start value
double waterTemp = 25;
double oldWaterTemp = 25; //for simple smoothing
unsigned long rpm = 0;
boolean fanRunningState = 1; //TODO: consider removing
int pwmstate = 255;
unsigned int pwmstateavg = 255;
unsigned long logicMillis = millis();
unsigned long reportMillis = 0; //reporting done separate interval

String inputString = "";         // a String to hold incoming data
boolean stringComplete = false;  // whether the string is complete
int manualSpeed = 0;  //value for set manual speed. If not 0 skips millis code.

void setup() {
    pinMode(TACHOPIN, INPUT);//_PULLUP);
    attachInterrupt(digitalPinToInterrupt(TACHOPIN), fanInterrupt, CHANGE);
    pinMode(PWMPIN, OUTPUT);
    TCCR2B = TCCR2B & B11111000 | B00000001; // for PWM frequency of 31372.55 Hz - sets pin 3&11
    pinMode(TEMPPINAIR, INPUT); 
    pinMode(TEMPPINWATER, INPUT); 

    //TCCR0B = TCCR0B & 0b11111101 | 0x01; //increases PWM frequency
    Serial.begin(115200); 
    delay(500); 
    Serial.println("Fancontroller boot");

    //run fan on max to indicate not braindead
    analogWrite(PWMPIN, 255);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(4000);

    inputString.reserve(3); //reserve for 3 character serial input (number=fan speed, 0-255)
    //Serial.println("Entering loop");
} //end setup()

void loop() {
    //serial input handler, If run overrides millis autimatics below
    if (stringComplete) {        
        manualSpeed = inputString.toInt();
        Serial.print("DBG: Manual Speed:");
        Serial.println(manualSpeed);
        analogWrite(PWMPIN, manualSpeed);
        inputString = ""; // clear the string:
        stringComplete = false;
    }

    //millis checks below
    unsigned long currentMillis = millis();
    if (currentMillis - logicMillis > 2000 and manualSpeed == 0) { //only do if not received manual speed
        logicMillis = millis();
        airTemp = calculateTemp(analogRead(TEMPPINAIR));
        waterTemp = calculateTemp(analogRead(TEMPPINWATER));
        waterTemp = (oldWaterTemp*1 + waterTemp) / 2; //simple smoothing
        oldWaterTemp = waterTemp; //save for next smoothing
        
        if (waterTemp < 0) waterTemp=0; //HACK! if negative values highly likely termistor broken. Set to 0 to avoid bad graphs at receiver.

        //---------calculate PWM want section-------------
        if (waterTemp >= MAXTEMP or waterTemp <= 0) { //full blast if over temp or termistor is bad (0 or lower)
            pwmstate = 255;    //past max temp, full power
            pwmstateavg = 255;
        } else if (waterTemp > MINTEMP) {
            if (fanRunningState == 0 or rpm < MINRPM) { //kick pwmstateavg high to get fan running. TODO: Consider phasing out runningstate since rpm consistent
                Serial.print("DBG: Kick, fr: ");
                Serial.print(fanRunningState);
                Serial.print(", rpm: ");
                Serial.println(rpm);
                fanRunningState = 1;
                digitalWrite(LED_BUILTIN, HIGH);
                pwmstateavg = KICKSPEED;
                pwmstate = KICKSPEED;
                analogWrite(PWMPIN, KICKSPEED); //probably pointless since set again below.
            } else {
              //map temp to PWM, using temp*100 to handle decimals since map function below does integer math
              pwmstate = map(waterTemp*100, MINTEMP*100, MAXTEMP*100, MINSPEED, 255); //dynamically adjust PWM based on temp range. 
            }  
        } else if (waterTemp < MINTEMP) pwmstate = 0; //fast drop to fan off (still averaged below). 
        pwmstateavg = (pwmstateavg*3 + pwmstate) / 4; //averaging. var 3/4

        //---------set PWM want section, pwmstateavg is want-------------
        if (pwmstateavg > MINSPEED or (pwmstate > MINSPEED and pwmstateavg > MINSPEED)) {
            analogWrite(PWMPIN, pwmstateavg);
        } else {
            analogWrite(PWMPIN, 0);
            digitalWrite(LED_BUILTIN, LOW);
            fanRunningState = 0;
        }
    } //END logicMillis
        
    if (currentMillis - reportMillis > 30000 and manualSpeed == 0) { //only do if not received manual speed
        reportMillis = millis();        
        Serial.print("Ta: ");
        Serial.print(airTemp);
        Serial.print(" Tw: ");
        Serial.print(waterTemp);
        Serial.print(" Pc: ");
        Serial.print(pwmstate);
        Serial.print(" Pa: ");
        Serial.print(pwmstateavg);
        Serial.print(" Fr: ");
        Serial.print(fanRunningState);
        Serial.print(" RPM: ");
        Serial.println(rpm);     

        rpm = 0; //reset rpm counter
    } //END reportMillis
} //end loop()

void fanInterrupt() { //pin2 interrupt handler
    rpm++;
}

void serialEvent() { //Serial input handler.
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

double calculateTemp(int RawADC) {
    double Temp;
    Temp = log(10000.0 * ((1024.0 / RawADC - 1)));
    //         =log(10000.0/(1024.0/RawADC-1)) // for pull-up configuration
    Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
    Temp = Temp - 273.15;            // Convert Kelvin to Celcius
    return Temp;
}


