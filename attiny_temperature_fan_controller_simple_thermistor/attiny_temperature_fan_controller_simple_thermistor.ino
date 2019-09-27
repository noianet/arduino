/*
 Simple temperature driven fan controller. PWM range.
 Using unknown thermistors from LIPO pack
  
  //OPTION/TODO: Overheat relay?
*/
//#define DEBUG //if uncommented, uses serial debugging. Remember 8Mhz minimum

#include <math.h>

#define MAXTEMP 1800 //Fan on full speed. Millivolt. Decrease with temperature. 1300 roughly 65C
#define MINTEMP  2200 //FAN off. Millivolt. Roughly 40C (was 2000)
#define MINSPEED 20 //Minimum fan speed, Referenced PWM range 0-255
#define TEMPSENSORPIN1 A1 //physpin 7, 
//#define TEMPSENSORPIN2 A2 //physpin 3, A2, PB4.  NB - correct below if changed
#define PWMPIN 0//physpin 5. NB - correct below if changed
#define VCC_OFFSET -690 //Offset VCC pinn read vs voltmeter measurement, millivolt.
#define DEBUG//using Pin PB3 physpin2. Debug data, temperature read    

#ifdef DEBUG
#include <TinyDebugSerial.h>
TinyDebugSerial mySerial = TinyDebugSerial();
#endif

int voltage = 0;
int pwmstate = 255;
float pwmstateavg = 255.0;

void setup() {
    //Output low on unused pins for noise suppression.
    pinMode(PWMPIN, OUTPUT); //physpin 5, PB0, D0
    digitalWrite(0,LOW);
    pinMode(1, OUTPUT); //physpin 6, PB1 (PWM able)
    digitalWrite(1,LOW);
    pinMode(TEMPSENSORPIN1, INPUT); //physpin 7, PB2, A1
#ifndef DEBUG //using for serial. Pull down if not used (ndef)
    pinMode(3, OUTPUT); //physpin 2, PB3/A3, ADC3 (PWM able)
    digitalWrite(3,LOW);
#endif
//    pinMode(TEMPSENSORPIN2, INPUT); //physpin 3, PB4/A2, ADC2,  (PWM able)
    pinMode(5, OUTPUT); //physpin 1, PB5/A0, ADC0
    digitalWrite(5,LOW);
    
    TCCR0B = TCCR0B & 0b11111101 | 0x01; //increases PWM frequency
#ifdef DEBUG
   mySerial.begin( 9600 ); //NB: With PWM change 9600=19200 on receiver. Using Pin PB3 physpin2. 
   mySerial.println("ATTINY boot");
#endif
   //run fan to indicate not braindead
   analogWrite(PWMPIN, 255);
   delay(128000); //2sec with fast pwm
}

void loop() {
        //temperature = analogRead(TEMPSENSORPIN) * AREF * 0.1 - 50.0; // value to celcius conversion for TMP36
        //float vccvoltage = readVcc() + VCC_OFFSET; 
        //vccvoltage /= 1000; //reduce from milivolt
        float vccvoltage = 5.00;
        
       float thermistor1 = analogRead(TEMPSENSORPIN1)  * vccvoltage;
       //float thermistor2 = analogRead(TEMPSENSORPIN2)  * vccvoltage;
      //use lowest voltage/warmest thermistor
      //if (thermistor1 > thermistor2) voltage = thermistor2; else (voltage=thermistor1);
      voltage=thermistor1;

       //Calculate want PWMstate. Remember lower voltage higher temp.
        if (voltage <= MAXTEMP) {pwmstate = 255; pwmstateavg = 255;} //past max temp, full power
        else if (voltage < MINTEMP) {
            pwmstate=map(voltage,MINTEMP,MAXTEMP,MINSPEED,255); //dynamically adjust PWM based on temp range. Inverse
        }
        else if (voltage > MINTEMP) pwmstate = 0; //fan off. -2 to have some more hysterisis around off point

       pwmstateavg=(pwmstateavg*9+pwmstate)/10; //averaging. Booting with pwmstateavg=255
       if (pwmstateavg > MINSPEED or (pwmstate > MINSPEED and pwmstateavg > MINSPEED)) {
          analogWrite(PWMPIN, (int)pwmstateavg);
       } else analogWrite(PWMPIN, 0);

#ifdef DEBUG
        mySerial.print("VCC: ");
        mySerial.print(vccvoltage);
        mySerial.print(" T1: ");
        //mySerial.print(thermistor1);
        mySerial.print(voltage);
        //mySerial.print(" T2: ");
        //mySerial.print(thermistor2);
        mySerial.print(" PWMc: ");
        mySerial.print(pwmstate);
        mySerial.print(" PWMa: ");
        mySerial.println((int)pwmstateavg);
#endif
        delay(164000); //1s blocking cause why not. Lets call it a "hysterisis helper". Since increased PWM clock must be high

} //end loop()

/* no workie?
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
*/


