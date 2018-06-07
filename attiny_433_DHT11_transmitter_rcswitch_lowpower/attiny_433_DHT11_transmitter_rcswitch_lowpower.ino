//NB: Remember bootloader, 8MHz. If no confirmation set fuses (removes divide by 8):
//avrdude -c arduino -b 19200 -P /dev/ttyUSB0 -p attiny85 -U lfuse:w:0xe2:m -U hfuse:w:0xdd:m -U efuse:w:0xfe:m -B 20
//hfuse  DD (Brownout detection level1 2.8V)
//efuse  FE (self programming enable)

#include <dht.h> //From Rob Tillaart
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>
#include <RCSwitch.h>

RCSwitch radio = RCSwitch();
dht DHT11;

#define SLEEPLOOP 75 //sleep loops before check/transmit. x*8s. Set to 75 in prod
#define VCCREPORT 3 //report interval for battery voltage. Counter goes up 1 every tempreport.
#define DHTPIN PB3

#define CHECKLOOPMAX 10
float humidity=10;  //Stores humidity value
float temperature=10; //Stores temperature value
float oldhumidity;  //Old value for sanitychecks
float oldtemperature; //Old values for sanitycheck
float oldvoltage; //Old values for sanitycheck
int tempcounter = SLEEPLOOP; //sleep loop counter, set high to force immediate read on boot.
int vcccounter = VCCREPORT; //initially set high to trigger battery voltage report on boot.

void setup() {
    // Transmitter is connected to Attiny Pin PB0,
    radio.enableTransmit(PB0);
    setup_watchdog(9); // enable watchdog service. (9) = 8s (max sleep interval)

    //Enable internal pullups on unused pins for noise suppression.
    //PB0 used, 433MHz transmitter
    pinMode(PB1, INPUT_PULLUP);
    pinMode(PB2, INPUT_PULLUP);
    //PB3 used, DHT11
    pinMode(PB4, INPUT_PULLUP);
    pinMode(PB5, INPUT_PULLUP); //PB5 (reset) with extra external pullup


    int chk = DHT11.read11(DHTPIN); //Populate some data
    oldtemperature = DHT11.temperature ;
    oldhumidity = DHT11.humidity;

     oldvoltage= readVcc(); //populate initial value
       
    //indicate startup
    delay(1000);
    radio.send(99, 24); //indicate boot
    delay(1000);
}

void loop() {
    if (tempcounter < SLEEPLOOP) {
        tempcounter++;
        sleep();
    } else {
        //sanitychecks,
        for (int t=0; t < CHECKLOOPMAX; t ++)  {
            int chk = DHT11.read11(DHTPIN);
            temperature = DHT11.temperature ;
            humidity = DHT11.humidity;
            if ( (temperature - oldtemperature < 10.0 &&  temperature - oldtemperature > -10.0 ) || t == CHECKLOOPMAX ) { //if sane value or at max loops accept reading
                oldtemperature = temperature;
            }
            if ( (humidity - oldhumidity < 10.0 &&  humidity - oldhumidity > -10.0) || t == CHECKLOOPMAX ) { //if sane value or at max loops accept reading
                oldhumidity = humidity;
            }
            if ( temperature == oldtemperature && humidity == oldhumidity ) { //if readings was OK, accept and break out, else re-read
                break;
            } 
            sleep(); //wait a bit before next read attempt
        } //END for loop sanitychecks.

        //radio transmission part.
        if (temperature >= 0) { //if to handle pos/negative values prefixing
            radio.send(900000 + temperature*100, 24); //times 100 for decimals, reduced at receiver
        } else { //negative temp, change to positive number and use 21 prefix
            temperature = fabsf(temperature);
            radio.send(910000 + temperature*100, 24); //times 100 for decimals, reduced at receiver
        }
        delay(1000); //allow the first transmission to complete before the next one.
        if (humidity > 0) { //to avoid sending garbage on startup...
            radio.send(920000 + humidity*100, 24);  //times 100 for decimals, reduced at receiver
        }

        //report VCC voltage (milivolt) - run only once every VCCREPORT temp reports
        if (vcccounter < VCCREPORT) {
            vcccounter++;
        } else {
            delay(1000); //allow the first transmission to complete before the next one.
            float vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
            //check to avoid spikes - seems to be rare
            if (vccvoltage - oldvoltage > 1000.0 &&  vccvoltage - oldvoltage < -1000.0) { //if spike +1V re-read
              delay(1000);
              vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
              oldvoltage=vccvoltage;
            } else {
              oldvoltage=vccvoltage;
            }
            radio.send(980000 + vccvoltage, 24); 
            vcccounter = 1; //reset counter
        }
        //End radio transmission part
        //reset counter and go to sleep
        tempcounter = 1; //reset counter
        sleep();  //go to interrupt sleep
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

    //result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
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


