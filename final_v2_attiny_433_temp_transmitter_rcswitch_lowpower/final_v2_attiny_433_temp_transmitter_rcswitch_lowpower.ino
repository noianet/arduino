/*
    Using TMP36 sensor with internal ATTINY. External 3.3v  regulator for stability (internal 1v1 was not stable).
    Temperature calues are times 100 to get 2 decimal, reduce on receiver.
    Prefix'es for radio transmission in receiver script:
    90xxxx Normal  C
    91xxxx Negative C
    92xxxx VCCvoltage, result is milivolt
    97xxxx Debug flag - not used for now
    98xxxx Debug warning flag - not used for now
    99 Boot indicator.

*/
#define DEBUG //if uncommented, uses serial out instead of radio. NB: TX uses same pin as radio so switch in HW too.

#include <avr/sleep.h>
#include <avr/wdt.h>
#include <math.h>


#define TEMPSENSORPIN A2 //physpin 3, A2, PB4
#define SUPPLYVOLTAGE 1.1 //1.1V for internal reference. Adjust accordingly. Works best with 3.3v supply for ATTINY and TMP36
#define SLEEPLOOP 10 //sleep loops before check/transmit. x*8s. Set to 80 in prod
#define VCCREPORT 100 //report interval for battery voltage. Set to 100 for prod. Counter goes up 1 every tempreport.


#ifdef DEBUG
#include <TinyDebugSerial.h>
TinyDebugSerial mySerial = TinyDebugSerial();
#else
#include <RCSwitch.h>
RCSwitch radio = RCSwitch();
#endif

float temperature;
float oldtemp = 20;
int tempcounter = SLEEPLOOP; //sleep loop counter, set high to force immediate read on boot.
int vcccounter = VCCREPORT; //initially set high to trigger battery voltage report on boot.

void setup() {
#ifdef DEBUG
    mySerial.begin( 9600 ); //using Pin PB3 phys pin2 also for serial, so disable radio transmission if used.
#else
    // Transmitter is connected to Attiny Pin PB3, That is physical pin2
    radio.enableTransmit(3);
#endif


    setup_watchdog(9); // enable watchdog service. (9) = 8s (max sleep interval)

    analogReference(INTERNAL1V1); //use internal 1.1V reference for analog read. adjust SUPPLYVOLTAGE if activated.

    //Enable internal pullups on unused pins for noise suppression.
    pinMode(0, INPUT_PULLUP); //physpin 5. Not analog in/might be overkill
    pinMode(1, INPUT_PULLUP); //physpin 6. Not analog in/might be overkill
    pinMode(2, INPUT_PULLUP); //physpin 7
    //pin 3 used for tempsensor (physpin 2)
    //pin 4 used for radio or serial TX debugging (physpin 3).

    //indicate startup
    delay(1000);
#ifdef DEBUG
    mySerial.println("ATTINY boot");
#else
    radio.send(99, 24); //indicate boot
#endif
    delay(1000);
}

void loop() {
    if (tempcounter < SLEEPLOOP) {
        //mySwitch.send(27, 24); //DEBUG, send too indicate sleep tick.
        tempcounter++;
        sleep();
    } else {
        delay(200); //might help stabilize ADC..
        int pinread  = analogRead(TEMPSENSORPIN); //extra read to avoid ghost voltage
        pinread  = analogRead(TEMPSENSORPIN);
        //temperature = analogRead(TEMPSENSORPIN) * AREF * 0.1 - 50.0; // value to celcius conversion for TMP36
        float voltage = pinread * SUPPLYVOLTAGE / 1024; //convert reading to milivolt
        float temperature = (voltage - 0.5) * 100 ; //converting from 10 mv per degree.

        //average 2 reads. Times 100 to get 2 decimal points for transmission. Reduced at receiver
        int tempaverage = (temperature * 100 + oldtemp * 100) / 2;

#ifdef DEBUG
        //Debug. Enable tinySerial for output. NB: Uses radio transmit pin.
        float vccvoltage = readVcc();
        vccvoltage = vccvoltage / 1000; //reduce from milivolt
        mySerial.print("Pinread: ");
        mySerial.print(pinread);
        mySerial.print(" V_calc: ");
        mySerial.print(voltage, 3);
        mySerial.print(" VCC_calc: ");
        mySerial.print(vccvoltage, 3);
        mySerial.print(" T_calc: ");
        mySerial.print(temperature, 2);
        mySerial.print(" T_avrg: ");
        mySerial.println(tempaverage);
        //End debug part.
#else
        //radio transmission part.
        if (temperature == oldtemp) {
            //temp unchanged, do nothing
        } else if (tempaverage >= 0) { //if to handle pos/negative values prefixing
            radio.send(900000 + tempaverage, 24);
        } else { //negative temp, change to positive number and use 21 prefix
            tempaverage = fabsf(tempaverage);
            radio.send(910000 + tempaverage, 24);
        }
        oldtemp = temperature;

        //report VCC voltage (milivolt) - run only once every VCCREPORT temp reports
        if (vcccounter < VCCREPORT) {
            vcccounter++;
        } else {
            delay(500); //short wait for next radio transmission to stabilize vcc
            float vccvoltage = readVcc(); //get VCC voltage, result is milivolt.
            delay(10); //extra wait before radio mess
            radio.send(920000 + vccvoltage, 24);
            vcccounter = 1; //reset counter
        }
        //End radio transmission part
#endif

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


