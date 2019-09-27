//                                      +-\/-+
//                 Reset  PB5  1|o       |8  VCC, CE NRF24L01 (tied high), 4.7k pullup to PB4 (for AM2320)
//  CSN NRF24L01 PB3  2|         |7  SCK      PB2
//             sensor    PB4  3|         |6  MOSI    PB1 (nb: MISO/MOSI flipped from pinout since is master)
//                          GND  4|         |5  MISO    PB0 (nb: MISO/MOSI flipped from pinout since is master)
//                                     +----+
/*  Mapping:
    ATTINY  NRF24
    PB0   - MISO
    PB1   - MOSI
    PB2   - SCK
    PB3   - CSN (active low)
    PB4  -   DHTxx, or AM2320 single wire mode, with sensor datapin. Must have a 4.7k pullup if using AM2320.
    PB5 (Reset, port is set as input pullup to avoid floating). Shoud have an external 4.7 pullup as well from datasheet but not critical.

    Description:
    On powerup/interrupt (PB4, report voltage, then sleep until pin interrupt
    Leafswitch (NC-mailbox closed=open) attached to reset+gnd.
*/


#define NODE_ID 9 //==========================CHANGE FOR EACH SENSOR=====================

#include <avr/sleep.h>
//#include <avr/wdt.h>
#include <avr/power.h>
//#include <SPI.h>
#include <MySensor.h>
#define CHILD_ID_SWITCH 0
#define CHILD_ID_VOLTAGE 1
MySensor gw;
MyMessage msgSwitch(CHILD_ID_SWITCH, V_ARMED);
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);

#define PINS_H//Used in mysensors lib
#define NSS_NRF24 PB3 //Used in mysensors lib

void setup() {
    pinMode(PB4, INPUT_PULLUP); //Interupt pin, use pullup.
    pinMode(PB5, INPUT_PULLUP); //PB5 (reset)

    delay(1000); //wait before radio present (avoid battery bounce/multiple send)
    //init NRF24 with MySensors protocol
    SPI.begin();
    gw.begin(NULL, NODE_ID);
    //gw.sendSketchInfo("ATTiny_NRF24_mailbox", "1.0");
     gw.present(CHILD_ID_SWITCH, V_ARMED);
    gw.present(CHILD_ID_VOLTAGE, V_VOLTAGE);
    SPI.end(); //for consistency with loop
}

void loop() {
    delay(1000); //wait for stuff to stabilize
    float vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
    SPI.begin();
    gw.powerUp(); //radio on
    gw.send(msgSwitch.set(digitalRead(PB4), 2));
    gw.send(msgVolt.set((vccvoltage / 1000), 2));
    gw.powerDown(); //radio off
    SPI.end();
    deep_sleep();
} //END loop

//UTILS below:
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

    //scale_constant = internal1.1ref * 1024 * 1000
    //If large offset, recalculate internal1.1ref with: 1.1 * Vcc1 (per voltmeter) / Vcc2 (per readVcc() function)
    //result = 1074966L / result; //offset sensor 20 (0.2v too high): 1.04977*1024*1000

    //default reference, 1.1v:
    result = 1126400L / result; // Calculate Vcc (in mV); 1126400 = 1.1*1024*1000
    return result; // Vcc in millivolts
}

ISR(PCINT0_vect) {
    // This is called when the interrupt occurs, but I don't need to do anything in it
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

