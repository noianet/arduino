/*
    Pneumatic testjig with statemachine.
    Overview hardware:
    4x servos, 4x valves , 2x pressure sensors, 1x VL53L0X lidar. Single pneumatic cylinder.
    lidar lib doc: https://github.com/pololu/vl53l0x-arduino

    State machine logic:
    loop():
    Emergencypin, flush, and potmeter for manual control. Check for emergency/bleed switch.
    Potmeter pin ignored if 0 (ground pin for serial only), will override serial input if not..
    Normal mode jumps to runactuator() function.
    runactuator() function:
    Will run only if emergencypin pulled to ground, and for every loop read pressure and distance sensors, and compare with requested position,
    Will bleed chambers if necessary down to minimumpressure, and fill oposite chamber with what is necessary to reach its position. HYSTERISIS sets acceptable variation before running valves.
    "movespeed" variable adjusts valve opening angle from SERVOSTARTPOS, and can be controlled via serial input. Hard coced SERVORANGE for max opening angle from SERVOSTARTPOS.
    "wantpos" variable used for requested position. Can be controlled via POTMETERPIN (or if analogRead commented out in loop() directly via serial),
    No delays, millis used for serial output every LOG_PERIOD ms. Initial benchmarking on an Arduino nano gives 1300 loops/sec.
    stopactuator() function:
    Will run if emergencypin not pulled to ground. Sets all servos to -20 deg. Delay 1000ms with serial prompt,
    bleedactuator() function:
    Will only run if emergencypin and bleedswitch not pulled to ground. Sets all input servos to -20 deg and flush to defined BLEEDANGLE (10). Delay 1000ms with serial prompt,
    General IO:
    Serial input on interrupt, valid input are distance,speed, minimumpressure ex 200,30,120. Routine does input validation basend on constants.
    Overkill:
    I2C LCD support (adress 0x3F)

    TODO: Add compensating speed/motion calculations when close to wantpos to avoid overshoot if necessary. Need to test with HW, might use a variant of biased hysterisis.

    Remember to short emergency pin to ground to run actuator (internal pullup).
    Serial set at 115200.
*/
#include <Wire.h>
#include <VL53L0X.h> //https://github.com/pololu/vl53l0x-arduino
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#define LOG_PERIOD 1000  //Serial output logging period in milliseconds.

//PIN declarations:
#define EMERGENCYPIN 2 //internal pullup enabled, for emercency button (N/O button). Short to ground for normal operation.
#define BLEEDPIN 3 //internal pullup enabled, for switch. Only checked if emergency NOT shorted to ground. 
#define ESERVOPINFILL 6 //extend servo1
#define ESERVOPINFLUSH 7 //extend servo1 
#define RSERVOPINFILL 8 //retract servo1
#define RSERVOPINFLUSH 9 //retract servo1 
#define EPNEUMATICPIN A0 //pneumatic sensor extend line
#define RPNEUMATICPIN A1 //pneumatic sensor retract line
#define POTMETERPIN A2 //for manual position adjust
//lidar connected to I2C bus at A4 (SDA) and A5 (SCL)

//Physical definitions:
#define MINLENGTH 60 //min stroke position in mm. Is used as zero point. Related to position sensor positioning.
#define STROKELENGTH 370 //max stroke length in mm from MINLENGTH
#define EFILLSERVOSTARTPOS 5 //zero position for valve servos.
#define EFLUSHSERVOSTARTPOS 5 //zero position for valve servos. 
#define RFILLSERVOSTARTPOS 5 //zero position for valve servos.
#define RFLUSHSERVOSTARTPOS 5 //zero position for valve servos. 
#define SERVORANGE 80 //max movement above startpos (diff from startpos). 
#define BLEEDANGLE 20 //servo angle for bleed actuator, higher for quicker pressure release
#define HYSTERISIS 3 //hysterisis for position adjust. Larger value -> less small adjustments -> less power loss
#define REDUCEDSPEEDRANGE 30//Movespeed halved when inside +/- REDUCEDSPEEDRANGE from wantpos. Envelopes HYSTERISIS

//section definitions. Uncomment if connected. LCD cost: 1 loop/sec.
//#define HAVELCD

//servo declaratons for valve control
Servo eservofill;  // extend servo1
Servo eservoflush;  // extend servo2
Servo rservofill;  // retract servo1
Servo rservoflush;  // retract servo2

//define lidar
VL53L0X lidar;

//LCD init, yes actually...
LiquidCrystal_I2C lcd(0x3F, 20, 4); // set the LCD address to 0x3F for a 16 chars and 2 line display

//define and set some global variables
unsigned long previousMillis;  //variable for time measurement
unsigned int wantpos = 0; //adjusted by serial input and potmeter.
unsigned int movespeed = 15; //adjusted by serial input. Sets valve opening factor (servo angle from SERVOSTARTPOS).
unsigned int minimumpressure = 150; //adjusted by serial input., ideal working pressure. Atmospheric about 100 on test sensor. Set ex 50 to go atmospheric,
int havepos; //populated from lidar read;
int haveposlidar; //used for sanity check lidar rand debugging Had some large peaks.

//read some sensor values
int extendpressure = analogRead(EPNEUMATICPIN);
int retractpressure = analogRead(RPNEUMATICPIN);
int counter = 0; //for benchmark

void setup()
{
    Serial.begin(115200);
    Serial.setTimeout(10); //avoid long blocking wait for serial event.
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    //Wire.setClock(100000);

#ifdef HAVELCD
    lcd.init();
    lcd.backlight();
    lcd.setCursor(4, 0);
    lcd.print("Startup");
#endif
    //attach servos and set to closed position
    eservofill.attach(ESERVOPINFILL);
    eservoflush.attach(ESERVOPINFLUSH);
    rservofill.attach(RSERVOPINFILL);
    rservoflush.attach(RSERVOPINFLUSH);
    stopactuator(); //sets all servos to closed position

    //initialise lidar
    Serial.print("Probe I2C VL53L0X: ");
    lidar.init();
    Serial.println(lidar.readRangeSingleMillimeters() ); 
    if (lidar.readRangeSingleMillimeters() > 0) {
        lidar.setTimeout(500);
        havepos =lidar.readRangeSingleMillimeters() - MINLENGTH;
        Serial.print("Lidar found, measured range: ");
        Serial.println(havepos);
    } else {
#ifdef HAVELCD
        lcd.clear();
        lcd.print("!LIDAR MISSING!");
#endif
        Serial.println("ALERT: Lidar missing. Halting");
        Serial.print("Probe I2C VL53L0X result: ");
        Serial.println(lidar.readRangeSingleMillimeters()) ;
        while (1) { //blink onboard LED to indicate stop/error
            digitalWrite(13, HIGH);
            delay(200);          
            digitalWrite(13, LOW);
            delay(200);
        }
    }

    //enable pullup for emergency read and bleed switch. Short emergency to ground to run actuator.
    pinMode(EMERGENCYPIN, INPUT_PULLUP);
    pinMode(BLEEDPIN, INPUT_PULLUP);

    //flash onboard led to indicate setup done
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    lcd.setCursor(3, 0);
    lcd.print("Setup done");
    delay(1500);
    digitalWrite(13, LOW);
    Serial.println("Arduino setup complete");
}

void loop()
{
    unsigned long currentMillis = millis(); //time concept

    //read potmeter, translate to want position (adjust speed via serial). Ignore if 0.
    if (analogRead(POTMETERPIN) > 10) { //somewhat larger than 0 to avoid noise read issues
        wantpos = map(analogRead(POTMETERPIN), 0, 1023, 0, STROKELENGTH); // scale to stroke length
    }

    //check if emergency, pull to ground for actuator run (internal pullup enabled)
    //TODO: add to if statement " or lidar.timeoutOccurred()" - temp removed to work around missing lidar
    if (digitalRead(EMERGENCYPIN)) { //if emergency pin NOT shorted to ground or no respinse from lidar run stopactuator.
        if (digitalRead(BLEEDPIN)) {
            bleedactuator(); //close input valves and eject chambers if EMERGENCYPIN and BLEEDPIN NOT shorted to ground.
            Serial.println("Bleedactuator, input valves closed and flush open. Script delay 1s");
            lcd.clear();
            lcd.print("Emergency stop");
            lcd.setCursor(0, 1);
            lcd.print("Bleed chambers");
            delay(1000); //to avoid serial spam
        } else {
            stopactuator(); //only close all valves if bleedpin still shorted to ground
            Serial.println("Stopactuator. Script delay 1s");
            lcd.clear();
            lcd.print("Emergency stop");
            delay(1000); //to avoid serial spam
        }
    } else { //normal operation
        runactuator(); //run PID statemachine.

        //Serial output every LOG_PERIOD milliseconds
        if (currentMillis - previousMillis > LOG_PERIOD) {
            previousMillis = currentMillis;
            Serial.print("lps: ");
            Serial.print(counter);
            Serial.print(" spd: ");
            Serial.print(movespeed);
            Serial.print(" eprs: ");
            Serial.print(extendpressure);
            Serial.print(" rprs: ");
            Serial.print(retractpressure);
            Serial.print(" minprs: ");
            Serial.print(minimumpressure);
            Serial.print(" wantp: ");
            Serial.print(wantpos);
            Serial.print(" havep: ");
            Serial.println(havepos);

#ifdef HAVELCD
            lcd.clear();
            lcd.print("Pos:");
            //lcd.setCursor(5,0);
            lcd.print(havepos);
            lcd.setCursor(9, 0);
            lcd.print("W:");
            lcd.print(wantpos);
            lcd.setCursor(0, 1);
            lcd.print("ep:");
            lcd.print(extendpressure);
            lcd.setCursor(8, 1);
            lcd.print("rp:");
            lcd.print(retractpressure);
#endif
            counter = 0;
        }
        counter++;
    }
}
//----------------------------------------END INIT/SETUP/LOOP, utility functions below-----------------------------------------------------

void runactuator() { //simple statemachine, operates by calls to position reads and global vars for wants/speeds
    //sanity check for movespeed input, do not go beyond defined max open. (will happen if movespeed is set too high)
    if (movespeed > SERVORANGE) {
        movespeed = SERVORANGE;
    }

    //read some sensor values
    //DEBUG/TODO: temp lock to bypass missing lidar. Remove comment==================================================
    haveposlidar = lidar.readRangeSingleMillimeters(); //read distance, adjusted for defined minimum length to set zero point. Own variable for debugging
    if (haveposlidar > (MINLENGTH + STROKELENGTH + 50)) haveposlidar = MINLENGTH + STROKELENGTH; //sanitycheck lidar. Cap if way off
    havepos = haveposlidar  - MINLENGTH;
    //DEBUG/TODO: temp lock to bypass missing lidar==================================================
    extendpressure = analogRead(EPNEUMATICPIN); //pressure on extend line
    retractpressure = analogRead(RPNEUMATICPIN); //pressure on retract line

    // Try to hold position and a minimumpressure (to not vent down to atmosphere).
    // if position requires pressure below minimumpressure, hold valves closed (opposite direction will compensate)
    //Extend section (wantpos>havepos beyond hysterisis)
    if (((havepos + HYSTERISIS) < wantpos) && (havepos < STROKELENGTH)) { //need to extend. Check to avoid beyond max set by STROKELENGTH
        rservofill.write(RFILLSERVOSTARTPOS); //close input on retract line
        eservoflush.write(EFLUSHSERVOSTARTPOS); //close flush on extend line
        //check if above minimum pressure on retract line, eject air if is, else close
        if (retractpressure > minimumpressure && (wantpos - havepos) > REDUCEDSPEEDRANGE) { //move at indicated speed, outside REDUCEDSPEEDRANGE and flush at OK pressure.
            eservofill.write(EFILLSERVOSTARTPOS + movespeed);
            rservoflush.write(RFLUSHSERVOSTARTPOS + movespeed);
        } else if (retractpressure > minimumpressure && (wantpos - havepos) < (REDUCEDSPEEDRANGE / 1.5)) { //inside REDUCEDSPEEDRANGE, half speed. Flush at OK pressure but halfway to pos, set 1/4 flush
            eservofill.write(EFILLSERVOSTARTPOS + movespeed / 2);
            rservoflush.write(RFLUSHSERVOSTARTPOS + movespeed / 4);
        } else if (retractpressure > minimumpressure && (wantpos - havepos) < REDUCEDSPEEDRANGE) { //inside REDUCEDSPEEDRANGE, half speed. Flush at OK pressure
            eservofill.write(EFILLSERVOSTARTPOS + movespeed / 2);
            rservoflush.write(RFLUSHSERVOSTARTPOS + movespeed / 2);
        } else if (retractpressure <= minimumpressure && (wantpos - havepos) < REDUCEDSPEEDRANGE) { //inside REDUCEDSPEEDRANGE, half speed. Flush at low pressure (close)
            eservofill.write(EFILLSERVOSTARTPOS + movespeed / 2);
            rservoflush.write(RFLUSHSERVOSTARTPOS);
        } else {  //Else catches move at indicated speed, outside REDUCEDSPEEDRANGE, but flush at low pressure (close)
            eservofill.write(EFILLSERVOSTARTPOS + movespeed);
            rservoflush.write(RFLUSHSERVOSTARTPOS);
        }

        //Retract section
    } else if (((havepos - HYSTERISIS) > wantpos) && (havepos > 0)) { //need to retract, check to avoid going below zero point defined by MINLENGTH
        eservofill.write(EFILLSERVOSTARTPOS); //close input on extend line
        rservoflush.write(RFLUSHSERVOSTARTPOS); //close flush on retract line
        //check if above minimum pressure on extend line, eject air if is, else close
        if (extendpressure > minimumpressure && (havepos - wantpos) > REDUCEDSPEEDRANGE) { //move at indicated speed, outside REDUCEDSPEEDRANGE and flush at OK pressure.
            rservofill.write(RFILLSERVOSTARTPOS + movespeed);
            eservoflush.write(EFLUSHSERVOSTARTPOS + movespeed);
        } else if (extendpressure > minimumpressure && (havepos - wantpos) < (REDUCEDSPEEDRANGE / 1.5)) { //inside REDUCEDSPEEDRANGE, half speed. Flush at OK pressure but halfway to pos, set 1/4 flush
            rservofill.write(RFILLSERVOSTARTPOS + movespeed / 2);
            eservoflush.write(EFLUSHSERVOSTARTPOS + movespeed / 4);
        } else if (extendpressure > minimumpressure && (havepos - wantpos) < REDUCEDSPEEDRANGE) { //inside REDUCEDSPEEDRANGE, half speed. Flush at OK pressure
            rservofill.write(RFILLSERVOSTARTPOS + movespeed / 2);
            eservoflush.write(EFLUSHSERVOSTARTPOS + movespeed / 2);
        } else if (extendpressure <= minimumpressure && (havepos - wantpos) < REDUCEDSPEEDRANGE) { //inside REDUCEDSPEEDRANGE, half speed. Flush at low pressure (close)
            rservofill.write(RFILLSERVOSTARTPOS + movespeed / 2);
            eservoflush.write(EFLUSHSERVOSTARTPOS);
        } else {  //Else catches move at indicated speed, outside REDUCEDSPEEDRANGE, but flush at low pressure (close)
            rservofill.write(RFILLSERVOSTARTPOS + movespeed);
            eservoflush.write(EFLUSHSERVOSTARTPOS);
        }
    } else { //all good, close valves
        stopactuator();
    }
} //end statemachine

void stopactuator() { //sets all servos to closed position.
    eservofill.write(EFILLSERVOSTARTPOS);
    eservoflush.write(EFLUSHSERVOSTARTPOS);
    rservofill.write(RFILLSERVOSTARTPOS);
    rservoflush.write(RFLUSHSERVOSTARTPOS);
}

void bleedactuator() { //release pressure (for shutdown)
    eservofill.write(EFILLSERVOSTARTPOS);
    eservoflush.write(EFLUSHSERVOSTARTPOS + BLEEDANGLE);
    rservofill.write(RFILLSERVOSTARTPOS);
    rservoflush.write(RFLUSHSERVOSTARTPOS + BLEEDANGLE);
}


void serialEvent() { //runs on interrupt hardware RX.
    while (Serial.available() > 0) {
        // look for the next valid integer in the incoming serial stream:
        wantpos = Serial.parseInt();
        movespeed = Serial.parseInt();
        minimumpressure = Serial.parseInt();
        //Echo received variables back to master for verification, which must then retransmit if error
        if (Serial.read() == '\n') {
            Serial.print(wantpos);
            Serial.print(",");
            Serial.print(movespeed);
            Serial.print(",");
            Serial.println(minimumpressure);
        }

        //sanitycheck on input:
        if (wantpos > (STROKELENGTH)) wantpos = STROKELENGTH;   //asked for position beyond max
        if (wantpos < 0) {
            wantpos = 0;   //asked for negative position
        }
        if (movespeed > SERVORANGE) {
            movespeed = SERVORANGE;   //asked for speed beyond max (also rechecked in runactuator() )
        }
        if (movespeed < 0) {
            movespeed = 0;   //asked for negative speed
        }
    }
}
