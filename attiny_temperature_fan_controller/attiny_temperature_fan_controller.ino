/*
 Simple temperature driven fan controller. PWM range.
 Using TMP36 sensor with 5V supply. 
  
  //OPTION/TODO: Overheat relay?
*/
//#define DEBUG //if uncommented, uses serial out instead of radio. NB: TX uses same pin as radio so switch in HW too.

#include <math.h>

#define MAXTEMP 60.0 //Fan on full speed
#define MINTEMP 30.0 //FAN off temp30
#define MINSPEED 20 //Minimum fan speed, Referenced PWM range 0-255
#define TEMPSENSORPIN A2 //physpin 3, A2, PB4.  NB - correct below if changed
#define PWMPIN 0//physpin 5. NB - correct below if changed
#define VCC_OFFSET -690 //Offset VCC pinn read vs voltmeter measurement, millivolt.
//#define DEBUG//using Pin PB3 physpin2. Debug data, temperature read    

#ifdef DEBUG
#include <TinyDebugSerial.h>
TinyDebugSerial mySerial = TinyDebugSerial();
#endif


float temperature = 0;
int pwmstate = 255;
int pwmstateavg = 255;

void setup() {
    //Output low on unused pins for noise suppression.
    pinMode(PWMPIN, OUTPUT); //physpin 5, PB0, D0
    digitalWrite(0,LOW);
    pinMode(1, OUTPUT); //physpin 6, PB1 (PWM able)
    digitalWrite(1,LOW);
    pinMode(2, OUTPUT); //physpin 7, PB2 
    digitalWrite(2,LOW);
#ifndef DEBUG //using for serial
    pinMode(3, OUTPUT); //physpin 2, PB3/A3, ADC3 (PWM able)
    digitalWrite(3,LOW);
#endif
    pinMode(TEMPSENSORPIN, INPUT); //physpin 3, PB4/A2, ADC2,  (PWM able)
    pinMode(5, OUTPUT); //physpin 1, PB5/A0, ADC0
    digitalWrite(5,LOW);
    
     TCCR0B = TCCR0B & 0b11111101 | 0x01; //increases PWM frequency
#ifdef DEBUG
   mySerial.begin( 9600 ); //using Pin PB3 physpin2. Debug data, temperature read    
   mySerial.println("ATTINY boot");
#endif
   //run fan to indicate not braindead
   analogWrite(PWMPIN, 255);
   delay(128000); //2sec with fast pwm
}

void loop() {
        //temperature = analogRead(TEMPSENSORPIN) * AREF * 0.1 - 50.0; // value to celcius conversion for TMP36
        float vccvoltage = readVcc() + VCC_OFFSET; 
        vccvoltage /= 1000; //reduce from milivolt
       float voltage = analogRead(TEMPSENSORPIN)  * vccvoltage;
       voltage /= 1024.0;       
     // now print out the temperature
      float temperature = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree with 500 mV offset
                                               //to degrees ((voltage - 500mV) times 100)

        //Calculate want PWMstate. 
        if (temperature >= MAXTEMP) {pwmstate = 255; pwmstateavg = 255;} //past max temp, full power
        else if (temperature > MINTEMP) {
            pwmstate=map(temperature,MINTEMP,MAXTEMP,MINSPEED,255); //dynamically adjust PWM based on temp
        }
        else if (temperature < MINTEMP) pwmstate = 0; //fan off. -2 to have some more hysterisis around off point

       pwmstateavg=(pwmstateavg*9+pwmstate)/10; //averaging. Booting with pwmstateavg=255
       if (pwmstateavg > MINSPEED or (pwmstate > MINSPEED and pwmstateavg > MINSPEED)) {
          analogWrite(PWMPIN, pwmstateavg);
       } else analogWrite(PWMPIN, 0);

#ifdef DEBUG
        mySerial.print("VCC: ");
        mySerial.print(vccvoltage);
        mySerial.print(" Vpin: ");
        mySerial.print(voltage);
        mySerial.print(" Temp: ");
        mySerial.print(temperature);
        mySerial.print(" PWM: ");
        mySerial.print(pwmstate);
        mySerial.print(" PWMa: ");
        mySerial.println(pwmstateavg);
#endif
        delay(64000); //1s blocking cause why not. Lets call it a "hysterisis helper". Since increased clock must be high

} //end loop()

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



