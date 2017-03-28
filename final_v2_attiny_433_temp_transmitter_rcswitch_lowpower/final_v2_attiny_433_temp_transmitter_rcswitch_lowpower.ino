/* 
reminder:
- Added averaging, can remove at receiver script
- Added vcc calculation
- added external vref. Tie pin 5 high with 10k resistor
- disabled fudge hack...
- removed wdt flag/if
*/ 
#include <RCSwitch.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>

//watchdog flag
int counter = 100; //high to force immediate read on boot.
float oldtemp = 22; //guess something... will influence first read at boot.

//correct to batt.
//float aref_voltage = 5.10;         
//float aref_voltage = 4.94;

RCSwitch mySwitch = RCSwitch();


void setup() {
   // Transmitter is connected to Attiny Pin PB3  <--
   // That is physical pin2
  mySwitch.enableTransmit(3);
  pinMode(A2, INPUT); //temp read analog. Let it float. PhysPin 3
  pinMode(1, OUTPUT); //used to power to temp sensor. PhysPin 6
  digitalWrite(1,LOW); //for consistency

  //tie unused to ground
 pinMode(5, OUTPUT); //physpin 1
 digitalWrite(5, LOW);
 pinMode(2, OUTPUT); //physpin 7
 digitalWrite(2, LOW);
 pinMode(0, OUTPUT); //physpin 5  
 digitalWrite(0, LOW);
  
  setup_watchdog(9); // enable watchdog service
  
  //indicate startup
  delay(1000);
  mySwitch.send(29, 24); //indicate boot
  delay(1000); //initial wait, just in case...
}
 
void loop() {
    if (counter < 4) { //sleep ticks. x*8s. set to 80 in prod
        //mySwitch.send(27, 24); //DEBUG, send too indicate sleep tick. 
        counter = counter + 1;
        sleep();
    } else {
        //sleep ticks finished, check and report temperature
        //turn on sensor, read, turn off.
        digitalWrite(1,HIGH);
        //delay(100); <--was too short
        sleep(); //let the thing stabilize for 8 sec.

        long vcc_mv=readVcc(); //calculate VCC, return is milivolt
        mySwitch.send(220000+vcc_mv, 24); //DEBUG <--------------------------------------------------------
        
        float reading=analogRead(A2); 
        delay(10);    
        digitalWrite(1,LOW);
        
        // converting that reading to voltage, Ref voltage set at top
        //float voltage = reading * aref_voltage; --old static thing, remove after test
        float voltage = reading * (vcc_mv/1000);
        voltage /= 1024.0; //converting reading to true 0-xV voltage
        //fudge factor...
        //voltage=voltage+0.11;
     
        float temperatureFL = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree with 500 mV offset to degrees ((voltage - 500mV) times 100)

        int temperature = round(temperatureFL*100); //times 100 to get 2 decimals. reduced on reciever
      
        //voltage=voltage*1000;//DEBUG
        //mySwitch.send(990000+voltage, 24); //DEBUG

        //averaging with previous reading
        int avrg = round((temperature + oldtemp) / 2);
        
        if (avrg == oldtemp) {
            //temp unchanged, do nothing
        } else if (avrg >= 0) { //if to handle pos/negative values prefixing
            mySwitch.send(200000+avrg, 24);
        } else { //negative temp, change to positive and use 21 prefix
            avrg=fabsf(avrg);
            mySwitch.send(210000+avrg, 24);
        }
        oldtemp = temperature;
        counter = 1; //reset counter
        sleep();  //go to interrupt sleep
    } 
}

//reference for this bit here: http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference. Following is ATTINY85 specific (change for different micro).
  ADMUX = _BV(MUX3) | _BV(MUX2);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring
 
  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both
 
  long result = (high<<8) | low;
 
  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  //nothing here, just go to loop
}


// Below routine to set system into the sleep state 
// system wakes up when wtchdog is timed out
void sleep() {
  ADCSRA &= ~_BV(ADEN);                // ADC off
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  ADCSRA |= _BV(ADEN);                 // ADC on
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;

  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}
  

