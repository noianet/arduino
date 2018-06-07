#include <RCSwitch.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

int counter = 1; //step counter

RCSwitch mySwitch = RCSwitch();
 
void setup() {
   // Transmitter data is connected to Attiny Pin PB3 (physpin 4)
  mySwitch.enableTransmit(3);
  
  //pin 4 (phys pin 3) for interrupt/awake
  pinMode(4, INPUT_PULLUP);

  //pin1 (Phys 6) for transmitter power
  pinMode(1, OUTPUT); //used to power boostconverter/transmitter
  digitalWrite(1,LOW); // start low to secure power to micro/stabilization

  //tie unused to ground
  pinMode(5, OUTPUT); //physpin 1
  digitalWrite(5, LOW);
  pinMode(2, OUTPUT); //physpin 7  
  digitalWrite(2, LOW);
  pinMode(0, OUTPUT); //physpin 5  
  digitalWrite(0, LOW);
  
  //indicate startup
  delay(1000);
  digitalWrite(1,HIGH); //enable boost converter
  delay(1000); //to stabilize votages if needed
  mySwitch.send(19, 24);  //indicate boot
  //does not turn off transmitter here, will be handled further down
  delay(1000); //extra wait, just in case...
  
  setup_watchdog(9); // enable watchdog service
}
 
void loop() {
    digitalWrite(1,HIGH); //enable boost/transmitter - will send something every round
    delay(600); //to stabilize votages if needed

    if (digitalRead(4) == LOW and counter ==1 ) { //normal reporting loop, open box. Don't check pin state to handle quick open/close
        mySwitch.send(10, 24);
        counter = counter + 1; //inc for next step
        digitalWrite(1,LOW); //turns off transmitter
        sleep();
    } else if (digitalRead(4) == LOW and counter == 2) { //lid still open, send warning
        mySwitch.send(11, 24);
        counter = counter + 1; //inc for next step
        digitalWrite(1,LOW); //turns off transmitter
        sleep();  
    } else if (digitalRead(4) == LOW) { //lid still open, send final warning and go to sleep
        mySwitch.send(12, 24);
        counter = 1; //reset counter
        digitalWrite(1,LOW); //turns off transmitter
        wdt_disable(); //disable watchdog timer
        deep_sleep();
    } else {
        //watchdog sleep/wake loop finised, go back to interrupt sleep again
        mySwitch.send(13, 24);//call home lid closed, going to sleep
        counter = 1; //reset counter
        digitalWrite(1,LOW); //turns off transmitter
        wdt_disable(); //disable watchdog timer
        deep_sleep();  //go to interrupt sleep
    } 
}

//pin interrupt service
ISR(PCINT0_vect)
{
    setup_watchdog(9); // enable watchdog service
} 

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
    //nothing special here, continue to loop
}


//interrupt sleep block
void deep_sleep() {
    GIMSK |= _BV(PCIE);                     // Enable Pin Change Interrupts
    PCMSK |= _BV(PCINT4);                   // Use PB4 as interrupt pin
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);    // replaces above statement

    sleep_enable();                         // Sets the Sleep Enable bit in the MCUCR Register (SE BIT)
    sei();                                  // Enable interrupts
    sleep_cpu();                            // sleep

    cli();                                  // Disable interrupts
    PCMSK &= ~_BV(PCINT4);                  // Turn off PB4 as interrupt pin
    sleep_disable();                        // Clear SE bit
    ADCSRA |= _BV(ADEN);                    // ADC on

    sei();                                  // Enable interrupts
}

// Below routine to set system into the sleep state 
// system wakes up when wtchdog is timed out
void sleep() {
    ADCSRA &= ~_BV(ADEN);                   // ADC off
    set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
    sleep_enable();
    sleep_mode();                        // System sleeps here
    sleep_mode();                        // System sleeps here
    sleep_disable();                     // System continues execution here when watchdog timed out 
    ADCSRA |= _BV(ADEN);                    // ADC on
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
  
//EOF
