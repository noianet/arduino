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
Simple, small and low power (14.5uAh sleep)  temp and humidity sensor. Using mysensors 1.4.x stack, but works fine with mysensors 2.2 gateways. 
Using wake up/transmit temp/humidity every 10 min, and reports voltage every 30. Battery voltage calculated using Attiny internal 1.1v reference.
*/


#define NODE_ID 24 //==========================CHANGE FOR EACH SENSOR=====================

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#define SLEEPLOOP 75 //75 //sleep loops before check/transmit. x*8s. Set to 75 in prod for ca 10 min
#define VCCREPORT 3 //report interval for battery voltage. Counter goes up 1 every tempreport. 
#define CHECKLOOPMAX  7 //Recheck if it spikes more than +/-10. Using sleep(), 8s between each read, and when max reached giving up/accepting values.

#include <MySensor.h>
#define CHILD_ID_TEMP 0
#define CHILD_ID_HUM 1
#define CHILD_ID_VOLTAGE 2
MySensor gw;
MyMessage msgTemp(CHILD_ID_TEMP, V_TEMP);
MyMessage msgHum(CHILD_ID_HUM, V_HUM);
MyMessage msgVolt(CHILD_ID_VOLTAGE, V_VOLTAGE);

#define PINS_H//Used in mysensors lib
#define NSS_NRF24 PB3 //Used in mysensors lib
#define DHTPIN PB4

#include <dht.h>
dht sensor; //using AM2320 in single wire mode.

float humidity = 0; //Stores humidity value
float temperature = 0; //Stores temperature value
float oldhumidity = 0; //Old value for sanitychecks
float oldtemperature = 0; //Old values for sanitycheck
float oldvoltage; //Old values for sanitycheck
int tempcounter = SLEEPLOOP; //sleep loop counter, set high to force immediate read on boot.
int vcccounter = VCCREPORT; //initially set high to trigger battery voltage report on boot.

void setup() {
    pinMode(PB5, INPUT_PULLUP); //PB5 (reset), enable internal pullup to avoid floating
    setup_watchdog(9); // enable watchdog service. (9) = 8s (max sleep interval)

    //init NRF24 with MySensors protocol
    SPI.begin();
    gw.begin(NULL, NODE_ID);
    gw.sendSketchInfo("ATTiny_NRF24_AM2320", "1.0");
    gw.present(CHILD_ID_TEMP, S_TEMP);
    gw.present(CHILD_ID_HUM, S_HUM);
    gw.present(CHILD_ID_VOLTAGE, V_VOLTAGE);
    SPI.end();

    delay(1000); //wait for stuff to stabilize to avoid some insane reads
    int chk = sensor.read(DHTPIN); //Populate some data
    oldtemperature = sensor.temperature ;
    oldhumidity = sensor.humidity;

    oldvoltage = readVcc(); //populate initial value
}

void loop() {
    if (tempcounter < SLEEPLOOP) {
        tempcounter++;
        sleep();
    } else {
        int chk = sensor.read(DHTPIN);
        temperature = sensor.temperature ;
        humidity = sensor.humidity;

        //sanitychecks:
        for (int t = 0; t < CHECKLOOPMAX; t ++)  {
            int chk = sensor.read(DHTPIN);
            temperature = sensor.temperature ;
            humidity = sensor.humidity;
            if ( (temperature - oldtemperature < 10 &&  temperature - oldtemperature > -10 ) || t == CHECKLOOPMAX ) { //if sane value or at max loops accept reading
                oldtemperature = temperature;
            }
            if ( (humidity - oldhumidity < 30 &&  humidity - oldhumidity > -30) || t == CHECKLOOPMAX ) { //if sane value or at max loops accept reading
                oldhumidity = humidity;
            }
            if ( (temperature == oldtemperature  && humidity == oldhumidity) ) { //if readings was OK, accept and break out, else re-read
                break;
            }
            sleep();; //wait a bit before next read attempt
        } //END for loop sanitychecks.

        SPI.begin();
        gw.send(msgTemp.set(temperature, 2));
        gw.send(msgHum.set(humidity, 2));
        //report VCC voltage only once every VCCREPORT temp reports
        if (vcccounter < VCCREPORT) {
            vcccounter++;
        } else {
            float vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
            //check to avoid spikes - seems to be rare
            if (vccvoltage - oldvoltage > 1000.0 && vccvoltage - oldvoltage < -1000.0) { //if spike +/-1V re-read
                delay(1000);
                vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
            }
            oldvoltage = vccvoltage;
            gw.send(msgVolt.set((vccvoltage / 1000), 2));
            vcccounter = 1; //reset counter
        }
        gw.powerDown(); //radio off
        SPI.end();

        //reset counter and go to sleep
        tempcounter = 1; //reset counter
        sleep();  //go to interrupt sleep
    }
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
    if (ii > 9 ) ii = 9;
    bb = ii & 7;
    if (ii > 7) bb |= (1 << 5);
    bb |= (1 << WDCE);
    ww = bb;

    MCUSR &= ~(1 << WDRF);
    // start timed sequence
    WDTCR |= (1 << WDCE) | (1 << WDE);
    // set new watchdog timeout value
    WDTCR = bb;
    WDTCR |= _BV(WDIE);
}
