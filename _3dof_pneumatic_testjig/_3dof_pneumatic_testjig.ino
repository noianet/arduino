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
       Will bleed chambers if necessary down to MINIMUMPRESSURE, and fill oposite chamber with what is necessary to reach its position. HYSTERISIS sets acceptable variation before running valves.
       "movespeed" variable adjusts valve opening angle from SERVOSTARTPOS, and can be controlled via serial input. Hard coced SERVORANGE for max opening angle from SERVOSTARTPOS.
       "wantpos" variable used for requested position. Can be controlled via POTMETERPIN (or if analogRead commented out in loop() directly via serial),
       No delays, millis used for serial output every LOG_PERIOD ms. Initial benchmarking on an Arduino nano gives 1300 loops/sec.
    stopactuator() function: 
        Will run if emergencypin not pulled to ground. Sets all servos to -20 deg. Delay 1000ms with serial prompt,
    bleedactuator() function:
        Will only run if emergencypin and bleedswitch not pulled to ground. Sets all input servos to -20 deg and flush to defined BLEEDANGLE (10). Delay 1000ms with serial prompt,
  General IO:
      Serial input on interrupt, valid input are distance,speed ex 200,30. Routine does input validation basend on constants.
  Overkill: 
      I2C LCD support (adress 0x3F)
  
  TODO: Add compensating speed/motion calculations when close to wantpos to avoid overshoot if necessary. Need to test with HW, might use a variant of biased hysterisis.
  
  Remember to short emergency pin to ground to run actuator (internal pullup).
  Serial set at 115200.
*/
#include <Wire.h>
#include <VL53L0X.h>
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
#define POTMETERPIN A6 //for manual position adjust
//lidar connected to I2C bus at A4 (SDA) and A5 (SCL)

//Physical definitions:
#define MINLENGTH 100 //min stroke position in mm. Is used as zero point. Related to position sensor positioning.
#define STROKELENGTH 2000 //max stroke length in mm from MINLENGTH
#define SERVOSTARTPOS 30 //zero position for valve servos. Physical adjust almost open in this position.NB: emergency set this at -10
#define SERVORANGE 60 //max movement above startpos (added from startpos)
#define BLEEDANGLE 10 //servo angle for bleed actuator, higher for quicker pressure release

//Pneumatics relate. This factor will control energy efficiency (too much or too little venting) and somewhat speed,
#define MINIMUMPRESSURE 1 //ideal working pressure. Atmospheric about 100 on test sensor. Set ex 50 to go atmospheric,
#define HYSTERISIS 10 //hysterisis for position adjust. Larger value -> less small adjustments -> less power loss

//servo declaratons for valve control
Servo eservofill;  // extend servo1
Servo eservoflush;  // extend servo2
Servo rservofill;  // retract servo1
Servo rservoflush;  // retract servo2

//define lidar
VL53L0X lidar;

//LCD init, yes actually...
LiquidCrystal_I2C lcd(0x3F,20,4);  // set the LCD address to 0x3F for a 16 chars and 2 line display

//define and set some global variables
unsigned long previousMillis;  //variable for time measurement
int wantpos=0; //adjusted by serial input and potmeter. 
int havepos=0;  //populate in case read fails
int movespeed=20;  //adjusted by serial input. Sets valve opening factor (servo angle from SERVOSTARTPOS).

//read some sensor values
int extendpressure=analogRead(EPNEUMATICPIN);
int retractpressure=analogRead(RPNEUMATICPIN);
int counter=0; //for benchmark

void setup()
{
  Serial.begin(115200);
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.setCursor(5,0);
  lcd.print("Startup");
  
  //attach servos and set to closed position
  eservofill.attach(ESERVOPINFILL); 
  eservoflush.attach(ESERVOPINFLUSH); 
  rservofill.attach(RSERVOPINFILL); 
  rservoflush.attach(RSERVOPINFLUSH); 
  stopactuator(); //sets all servos to closed position

  //initialise lidar
  Serial.print("Probe I2C VL53L0X: ");
  lidar.init();
  lidar.setTimeout(500);
  havepos=lidar.readRangeSingleMillimeters()-MINLENGTH;
//DEBUG/TODO: temp lock to bypass missing lidar==================================================
  havepos=1000; //DEBUG/TODO: temp lock to bypass missing lidar
//DEBUG/TODO: temp lock to bypass missing lidar==================================================
    if (havepos>0) {
      Serial.print("Lidar found, measured range: ");
      Serial.println(havepos);
    } else {
        lcd.clear();
        lcd.print("!LIDAR MISSING!");
        Serial.println("ALERT: Lidar missing. Halting");
        while (1) {}
    }
  
  //enable pullup for emergency read and bleed switch. Short emergency to ground to run actuator.
  pinMode(EMERGENCYPIN, INPUT_PULLUP);
  pinMode(BLEEDPIN, INPUT_PULLUP); 
  
  //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW); 
  lcd.setCursor(3,0);
  lcd.print("Setup done");
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
 if (digitalRead(EMERGENCYPIN)) { //if emergency pin NOT shorted to ground run stopactuator. Pin has enabled internal pullup.
    if (digitalRead(BLEEDPIN)) { 
      bleedactuator(); //close input valves and eject chambers if EMERGENCYPIN and BLEEDPIN NOT shorted to ground.
      Serial.println("Bleedactuator, input valves set -10 from startpos and flush open. Script delay 1s");
      lcd.clear();
      lcd.print("Emergency stop");
      delay(1000); //to avoid serial spam
    } else {
      stopactuator(); //only close all valves if bleedpin still shorted to ground
      Serial.println("Stopactuator, all valves set -10 from startpos. Script delay 1s");
      lcd.clear();
      lcd.print("Emergency stop");
      lcd.setCursor(0,1);
      lcd.print("Bleed chambers");
      delay(1000); //to avoid serial spam
    }
  } else { //normal operation
    runactuator(); //run PID statemachine.

    //Serial output every LOG_PERIOD milliseconds
    if(currentMillis - previousMillis > LOG_PERIOD){
      previousMillis = currentMillis;
      Serial.print("loops/sec:: ");
      Serial.print(counter);
      Serial.print(" speed: ");
      Serial.print(movespeed);
      Serial.print(" epresss: ");
      Serial.print(extendpressure);
      Serial.print(" rpresss: ");
      Serial.print(retractpressure);
      Serial.print(" wantpos: ");
      Serial.print(wantpos);
      Serial.print(" havepos: ");
      Serial.println(havepos);
      
      lcd.clear();
      lcd.print("Pos:");
      //lcd.setCursor(5,0);
      lcd.print(havepos);
      lcd.setCursor(9,0);
      lcd.print("W:");
      lcd.print(wantpos);
      lcd.setCursor(0,1);
      lcd.print("ep:");
      lcd.print(extendpressure);
      lcd.setCursor(8,1);
      lcd.print("rp:");
      lcd.print(retractpressure);
      counter=0;
    }
    counter++;
  }
} 
//----------------------------------------END INIT/SETUP/LOOP, utility functions below-----------------------------------------------------

void runactuator() { //simple statemachine, operates by calls to position reads and global vars for wants/speeds
  //sanity check for movespeed input, do not go beyond defined max open. (will happen if movespeed is set too high)
  if (movespeed > SERVORANGE) {movespeed=SERVORANGE;}

  //read some sensor values
//DEBUG/TODO: temp lock to bypass missing lidar. Remove comment==================================================
  //havepos=lidar.readRangeSingleMillimeters()-MINLENGTH; //read distance, adjusted for defined minimum length to set zero point. 
//DEBUG/TODO: temp lock to bypass missing lidar==================================================


  //if (havepos < 0) {havepos=0;} //should avoid negative position calculation? commented out for now.
  extendpressure=analogRead(EPNEUMATICPIN); //pressure on extend line
  retractpressure=analogRead(RPNEUMATICPIN); //pressure on retract line

  // Try to hold position and a MINIMUMPRESSURE (to not vent down to atmosphere). 
  // if position requires pressure below MINIMUMPRESSURE, hold valves closed (opposite direction will compensate)
  
  if (((havepos+HYSTERISIS) < wantpos) && (havepos < STROKELENGTH)) { //need to extend. Check to avoid beyond max set by STROKELENGTH
    rservofill.write(SERVOSTARTPOS); //close input on retract line
    eservoflush.write(SERVOSTARTPOS); //close flush on extend line
    eservofill.write(SERVOSTARTPOS+movespeed); //open fill on extend line
    if (retractpressure>MINIMUMPRESSURE) { //check if above minimum pressure on retract line, eject air if is, else close
      rservoflush.write(SERVOSTARTPOS+movespeed);   
     } else {
      rservoflush.write(SERVOSTARTPOS); 
     }  
  } else if (((havepos-HYSTERISIS) > wantpos) && (havepos > 0)) { //need to retract, check to avoid going below zero point defined by MINLENGTH
    eservofill.write(SERVOSTARTPOS); //close input on extend line
    rservoflush.write(SERVOSTARTPOS); //close flush on retract line
    rservofill.write(SERVOSTARTPOS+movespeed); //open fill on retract line
    if (extendpressure>MINIMUMPRESSURE) { //check if above minimum pressure on extend line, eject air if is, else close
      eservoflush.write(SERVOSTARTPOS+movespeed);   
     } else {
      eservoflush.write(SERVOSTARTPOS); 
     }  
  } else { //all good, close valves
    eservofill.write(SERVOSTARTPOS); 
    eservoflush.write(SERVOSTARTPOS);
    rservofill.write(SERVOSTARTPOS); 
    rservoflush.write(SERVOSTARTPOS);
  }
} //end statemachine

void stopactuator() { //sets all servos to a bit beyond closed position. startpos is just before opening.
  eservofill.write(SERVOSTARTPOS-10); 
  eservoflush.write(SERVOSTARTPOS-10);
  rservofill.write(SERVOSTARTPOS-10); 
  rservoflush.write(SERVOSTARTPOS-10);
}

void bleedactuator() { //slowly release pressure based on input (for shutdown)
  eservofill.write(SERVOSTARTPOS-10); 
  eservoflush.write(SERVOSTARTPOS+BLEEDANGLE);
  rservofill.write(SERVOSTARTPOS-10); 
  rservoflush.write(SERVOSTARTPOS+BLEEDANGLE);
}


void serialEvent() { //runs on interrupt hardware RX.
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    wantpos = Serial.parseInt();
    movespeed = Serial.parseInt();
    //Echo received variables back to master for verification, which must then retransmit if error
    if (Serial.read() == '\n') {
      Serial.print(wantpos);
      Serial.print(",");
      Serial.println(movespeed);
    }
    //sanitycheck on input:
    if (wantpos > (STROKELENGTH)) {wantpos=STROKELENGTH;} //asked for position beyond max (also rechecked in runactuator() )
    if (wantpos < 0) {wantpos=0;} //asked for negative position
    if (movespeed > SERVORANGE) {movespeed=SERVORANGE;} //asked for speed beyond max (also rechecked in runactuator() )
    if (movespeed < 0) {movespeed=0;} //asked for negative speed
  }
}
