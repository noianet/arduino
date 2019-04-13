/*
    Automated mechanical weight scale, reporting via radio (Mysensors protocol).
    Hardware: Beetle (Leonardo), NRF24 radio, geared motor with hall sensor, limit switch, L293D H-bridge motor driver.
    With M5 rod current resolution is roughly 220 hall sensors pulses pr. mm.

    Holds its odometer counter in eeprom(0) to survive loss of power. Holding switch down when powering on will initiate homing.

    Operation:
    Init, reverses untill motor stall detected, then forward until limit switch.  Then report (counter, voltage, statisonfo),
    Next sleep loops, retransmits every 10 min to match listener slot. After 1 hr. do new weight check. If switch still closed, reverse until free then forward again.
    If motor stuck unexpectedly, hold/report until switch change (for manual exit from hold).    

   Note: Only sending floats for compatibility with old version custom receiver relay

    Status info, reported on Child_ID 3:
    0: Reset odometer (reverse)
    1: Forward until switch on, then 3 report.
    2: Used if switch on when waking reverse to switch off, then do 1.
    3: Reporting odometer
    4: Sleep loops. Retransmit data in 10min intervals to catch a listener slot.
    9: Stuck
    99, 91, 92: Stuck handling statechange switch
    Runmode switching in wake check, switch interrupt section and stuck detection in all runmodes,

*/
//MySensors NRF24pins etc
//#define MY_DEBUG // Enable debug prints to serial monitor
//#define MY_BAUD_RATE 115200
#define MY_RADIO_NRF24
#define MY_RF24_CE_PIN A0 //Can be tied high to save pins/use f.ex LED_BUILTIN pin to keep lib happy.
#define MY_RF24_CS_PIN A1 //CSN
#define MY_PASSIVE_NODE // Enable passive mode
#define MY_NODE_ID 200 //<============================= NodeID
#include <MySensors.h>
#include <EEPROM.h>

// Initialize message
MyMessage odometer(0, V_DISTANCE);
MyMessage voltage(1, V_VOLTAGE);
MyMessage statusInfo(2, V_VAR1);


//L293D
#define interruptPin 1  //RX, hall sensor interrupt
#define switchpin 0  //TX, interrupt limit switch.
#define motorPinEnable 9
#define motorPin1  10
#define motorPin2  11
#define millisInterval  500 //ms, how often do checks when awake
#define motorWait 300 //delay to wait for motor spinup
#define sleepTimer 60 //55 //56 //57=+1.2
#define retransmitInterval 10 //retransmit every x min. Subtracting each 60sec loop. Used in sleep and stuck.
float Odometer = 0; //float for compatibility Mysensor mymessage. Long would be OK othervise
float previousOdometer = 0;
byte runMode = 0; //0=init/reverse, 1: forward to switch, 2, reverse to switch off.
unsigned long currentMillis  = millis();
unsigned long previousMillis = 0;
int sleepCounter = 0;
int retransmitCounter = retransmitInterval;
float batteryvolt = 0;


//This will run only one time.
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    //Set pins as outputs
    //Serial.begin(115200);
    // Serial.println ();
    delay(5000); //anti-bricking..
    digitalWrite(LED_BUILTIN, LOW);
    //  Serial.println ("Setup");

    pinMode(interruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(interruptPin), hallsensorInterrupt, RISING);
    pinMode(switchpin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(switchpin), switchInterrupt, CHANGE);

    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPinEnable, OUTPUT);
    digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, LOW); //safety, power off
    digitalWrite(motorPinEnable, LOW); //safety, disable h-bridge, ensure led is off
    batteryvolt = readVcc();
    send(voltage.set(batteryvolt,2)); //Mysensors radio packet

    //test if homing selected (switch down on powerup), else read odometer value from eeprom and do runmode 1
    if (digitalRead(switchpin) == LOW) { 
        send(statusInfo.set(0.0,1)); //Mysensors statuschange radio packet, Float format for compatibility
        runMode = 0; //report
    } else {
      EEPROM.get(0,Odometer);
      send(odometer.set(Odometer,1)); //Mysensors radio packet  - mostly for sanitycheck eeprom read..
      send(statusInfo.set(1.0,1)); //Mysensors statuschange radio packet
      runMode = 1;
    }
}  //END setup

void presentation() {     // Mysensors, send the sketch version information to the gateway and Controller
    sendSketchInfo("Beehive Weight", "1.0");
    present(0, S_DISTANCE);
    present(1, S_MULTIMETER);
    present(2, S_CUSTOM);
}

void loop() {
    currentMillis = millis();
    if (runMode == 4) { //if in in sleep mode.
        //Serial.print("Run4 sleep, loop: "); Serial.println(sleepCounter);
        if (sleepCounter < sleepTimer) {
            sleepCounter++;
            digitalWrite(LED_BUILTIN, HIGH); //Flash to show something is happening
            delay(5);
            digitalWrite(LED_BUILTIN, LOW);
            //retransmit every 10 min to match listener slot if off.
            if (retransmitCounter <= 0) {
                send(odometer.set(Odometer,1)); //Mysensors radio packet
                send(voltage.set(batteryvolt,2)); //Mysensors radio packet
                send(statusInfo.set(4.0,1)); //Mysensors statuschange radio packet,
                retransmitCounter = retransmitInterval;
            } else {
                retransmitCounter--;
            }
            sleep(60000);
        } else { //passed sleep limit, wake up and do stuff:
            //USBDevice.attach(); //DEBUG. For serial reattach after sleep disconnect, comment out.
            digitalWrite(motorPinEnable, HIGH); //enable h-bridge.
            delay(1000); //might help radio wake.
            if (digitalRead(switchpin) == HIGH) {
                send(statusInfo.set(1.0,1)); //Mysensors statuschange radio packet,
                runMode = 1;   //if switch not triggered, forward. If already, reverse.
            } else {
                send(statusInfo.set(2.0,1)); //Mysensors statuschange radio packet,
                runMode = 2; //switch already on, reverse.
            }
        }

    } else { //not in sleep mode, do millis
        if (currentMillis - previousMillis > millisInterval) {
            previousMillis = currentMillis;
            digitalWrite(LED_BUILTIN, HIGH); //Flash to show something is happening
            delay(2);
            digitalWrite(LED_BUILTIN, LOW);

            //check runMode = 0; //0=init/reverse, 1: forward to switch, 2, reverse to fre switch, 3: report and switch off.
            //--------------Init, reverse untill stall detected and reset odometer----------------------------------
            if (runMode == 0) { //init, reverse until engin stalls.
                //Serial.print("Run0, Odo: "); Serial.println(Odometer);
                digitalWrite(motorPinEnable, HIGH); //enable h-bridge
                digitalWrite(motorPin1, HIGH);  digitalWrite(motorPin2, LOW);  //reverse
                delay(motorWait); ///give motor some time to spin up if just started.
                if (previousOdometer == Odometer) { //motor stalled
                    Odometer = 0;
                    if (digitalRead(switchpin) == HIGH) { //sanitycheck. Only continue if microswitch not triggered.
                        send(statusInfo.set(1.0,1)); //Mysensors statuschange radio packet,
                        runMode = 1;
                    } else {
                        send(statusInfo.set(9.0,1)); //Mysensors statuschange radio packet,
                        runMode = 9; //ERROR - something is stuck
                        retransmitCounter = retransmitInterval;
                    }
                } else {
                    previousOdometer = Odometer;
                }
            }

            //--------------Run forward until switchInterrupt----------------------------------
            else if (runMode == 1) {
                //Serial.print("Run1, Odo: "); Serial.println(Odometer);
                digitalWrite(motorPinEnable, HIGH); //enable h-bridge
                digitalWrite(motorPin1, LOW); digitalWrite(motorPin2, HIGH); //forward
                delay(motorWait); ///give motor some time to spin up if just started.
                if (previousOdometer == Odometer) { //motor stalled - might happen extending wagon too far/overweight.
                    send(statusInfo.set(9.0,1)); //Mysensors statuschange radio packet,
                    runMode = 9; //STUCK
                    retransmitCounter = retransmitInterval;
                } else {
                    previousOdometer = Odometer; //motor still running, update and continue
                }
            }
            //--------------Run reverse until switchInterrupt (release)-----------------------------------
            else if (runMode == 2) {
                //Serial.print("Run2, Odo: "); Serial.println(Odometer);
                digitalWrite(motorPinEnable, HIGH); //enable h-bridge
                digitalWrite(motorPin1, HIGH);  digitalWrite(motorPin2, LOW);  //reverse
                delay(motorWait); ///give motor some time to spin up if just started.
                if (previousOdometer == Odometer) { //motor stalled
                    send(statusInfo.set(9.0,1)); //Mysensors statuschange radio packet,
                    runMode = 9; //STUCK
                    retransmitCounter = retransmitInterval;
                } else {
                    previousOdometer = Odometer; //motor still running, update and continue
                }
            }
            //--------------Shutdown motor, report, sleep-----------------------------------
            else if (runMode == 3) { //report odometer value and go to sleep
                digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, LOW); //power off directio outputs
                //Serial.print ("Run3, Final odometer: "); Serial.println(Odometer);
                if (previousOdometer == Odometer) { //motor have spun down completely.
                    digitalWrite(motorPinEnable, LOW); digitalWrite(LED_BUILTIN, LOW); //disable h-bridge, ensure led is off
                    batteryvolt = readVcc();
                    send(odometer.set(Odometer,1)); //Mysensors radio packet
                    send(voltage.set(batteryvolt,2)); //Mysensors radio packet
                    send(statusInfo.set(4.0,1)); //Mysensors statuschange radio packet,
                    EEPROM.put(0, Odometer); //Store to eeprom in case power loss.
                    runMode = 4;
                    sleepCounter = 0;
                    retransmitCounter = retransmitInterval;
                } else {
                    previousOdometer = Odometer; //motor is still spinning down, wait for next milli
                }
            }
            //------------STUCK section, should not end below here. Hold until statechange switch.----------
            else {
                //Serial.print("RunX, STUCK: "); Serial.println(Odometer);
                digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, LOW); //power off directio outputs
                digitalWrite(motorPinEnable, LOW);//disable h-bridge
                digitalWrite(LED_BUILTIN, HIGH); //  extra flash to show something is happening
                delay(10);
                digitalWrite(LED_BUILTIN, LOW);
                if (retransmitCounter <= 0) {
                    send(statusInfo.set(9.0,1)); //Mysensors statuschange radio packet,
                    retransmitCounter = retransmitInterval;
                } else {
                    retransmitCounter--;
                }
                sleep(60000);
            } //end runmode if then else seq
        }//end millis 1s
    }//end if not in runmode sleep check
} //END LOOP


//==================UTILITIES BELOW=======================

void switchInterrupt() { //endstop triggered
    delay(100); // reduce bounce
    if (runMode == 1 and digitalRead(switchpin) == LOW) { //was running forward, switch runmode and reset previousMillis to force if sequence in loop,
        digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, LOW); //shut down motor2000
        //delay(2000); //allow time to spin down to limit miscounting
        send(statusInfo.set(3.0,1)); //Mysensors statuschange radio packet,
        runMode = 3; //report
        previousMillis = currentMillis; //reset millis to allow some spin down before entering do stuff
        //Serial.println("DEBUG - Switchpin1");
    } else if (runMode == 2 and digitalRead(switchpin) == HIGH) { //was on reverse to lift switch, next forward to trip.
        digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, LOW); //shut down motor
        //delay(2000); //allow time to spin down to limit miscounting
        send(statusInfo.set(1.0,1)); //Mysensors statuschange radio packet,
        runMode = 1; //Switch free, go forward
        previousMillis = currentMillis; //reset millis to allow some spin down before entering do stuff
        //Serial.println("DEBUG - Switchpin2"); 
    } else if (runMode == 9) { //STUCK, but someone pressed the button.
        digitalWrite(LED_BUILTIN, HIGH); //indicate registered
        digitalWrite(motorPinEnable, HIGH); //enable h-bridge
        delay(1000);
        digitalWrite(LED_BUILTIN, LOW);
        send(statusInfo.set(9.9,1)); //Mysensors statuschange radio packet,
        if (digitalRead(switchpin) == HIGH) {
            runMode = 1; //forward, trigger open
            digitalWrite(motorPin1, LOW);  digitalWrite(motorPin2, HIGH);
            send(statusInfo.set(9.1,1)); //Mysensors statuschange radio packet,
        } else {
            runMode = 2;
            digitalWrite(motorPin1, HIGH);  digitalWrite(motorPin2, LOW);
            send(statusInfo.set(9.2,1)); //Mysensors statuschange radio packet,
        }
        delay(motorWait); ///give motor some time to spin up if just started.
        //Serial.println("DEBUG - Switchpin3");
    }
}

void hallsensorInterrupt() { //if runmode motor direction forward (1), count up, else subtract.
    if (runMode == 1) {
        Odometer++;
    } else if (runMode == 0 or runMode == 2) { //Loosing some counts. Might be due to noise sensor powerup, trying to mitigate - DEBUG TESTING.
        Odometer--; 
    }
}

float readVcc() {
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
    result = result/1000; //convert from millivolts
    return result; // Vcc in volts
}


