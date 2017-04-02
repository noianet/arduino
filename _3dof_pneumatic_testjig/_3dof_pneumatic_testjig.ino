/* 
  pneumatic testjig with statemachine. 
  
  Remember to short emergency pin to ground to run actuator (internal pullup)
  
  TODO: adapt input to distance sensor when have access to hardware. Only analog read for now.
*/
#include <Servo.h>
#define LOG_PERIOD 1000  //Serial output logging period in milliseconds.

//PIN declarations:
#define EMERGENCYPIN 2 //internal pullup enabled, for emercency button (N/O button). Short to ground for normal operation.
#define BLEEDPIN 3 //internal pullup enabled, for switch. Only checked if emergency NOT shorted to ground. 
#define ESERVOPINFILL 6 //extend servo1
#define ESERVOPINFLUSH 7 //extend servo1 
#define RSERVOPINFILL 8 //retract servo1
#define RSERVOPINFLUSH 9 //retract servo1 
#define DISTANCEPIN A0 //distance sensor
#define EPNEUMATICPIN A1 //pneumatic sensor extend line
#define RPNEUMATICPIN A2 //pneumatic sensor retract line
#define POTMETERPIN A6 //for manual position adjust

//Physical definitions:
#define MINLENGTH 100 //min stroke position in mm. Is used as zero point. Related to position sensor positioning.
#define STROKELENGTH 1000 //max stroke length in mm from MINLENGTH
#define SERVOSTARTPOS 30 //zero position for valve servos. Physical adjust almost open in this position.NB: emergency set this at -10
#define SERVORANGE 130 //max movement above startpos (added from startpos)
#define BLEEDANGLE 10 //servo angle for bleed actuator, higher for quicker pressure release

//Pneumatics relate. This factor will control energy efficiency (too much or too little venting) and somewhat speed,
#define MINIMUMPRESSURE 100 //ideal working pressure. Atmospheric about 100 on test sensor.
#define HYSTERISIS 10 //hysterisis for position adjust. Larger value -> less small adjustments -> less power loss

#define SERIALSPEED 115200

//servo declaratons for valve control
Servo eservofill;  // extend servo1
Servo eservoflush;  // extend servo2
Servo rservofill;  // retract servo1
Servo rservoflush;  // retract servo2

//define and set some global variables
unsigned long previousMillis;  //variable for time measurement
int wantpos=MINLENGTH+50; //adjusted by serial input and potmeter. 
int movespeed=10;  //adjusted by serial input. Sets valve opening factor (servo angle from SERVOSTARTPOS). Careful since input also requires a wantpos

//read some sensor values
int havepos=analogRead(DISTANCEPIN); //read distance. TODO adapt to sensor. potmeter for test
int extendpressure=analogRead(EPNEUMATICPIN);
int retractpressure=analogRead(RPNEUMATICPIN);
int counter=0; //for benchmark

void setup()
{
  Serial.begin(SERIALSPEED);
  
  //attach servos and set to closed position
  eservofill.attach(ESERVOPINFILL); 
  eservoflush.attach(ESERVOPINFLUSH); 
  rservofill.attach(RSERVOPINFILL); 
  rservoflush.attach(RSERVOPINFLUSH); 
  stopactuator(); //sets all servos to closed position

  //enable pullup for emergency read and bleed switch. Short emergency to ground to run actuator.
  pinMode(EMERGENCYPIN, INPUT_PULLUP);
  pinMode(BLEEDPIN, INPUT_PULLUP); 
  
  //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW); 
  
  Serial.println("Arduino setup complete");
}

void loop() 
{ 
  unsigned long currentMillis = millis(); //time concept
  
  //read potmeter, translate to want position (adjust speed via serial - obs for required wantpos)
  wantpos = map(analogRead(POTMETERPIN), 0, 1023, 0, STROKELENGTH); // scale to stroke length
  
  //check if emergency, pull to ground for actuator run (internal pullup enabled)
 if (digitalRead(EMERGENCYPIN)) { //if emergency pin NOT shorted to ground run stopactuator. Pin has enabled internal pullup.
    if (digitalRead(BLEEDPIN)) { 
     bleedactuator(); //close input valves and eject chambers if EMERGENCYPIN and BLEEDPIN NOT shorted to ground.
    } else {
      stopactuator(); //only close all valves if bleedpin still shorted to ground
    }
  } else { //normal operation
    runactuator(); //run PID statemachine.

    //Serial output every LOG_PERIOD milliseconds
    if(currentMillis - previousMillis > LOG_PERIOD){
      previousMillis = currentMillis;
      Serial.print("bm:: ");
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
  havepos=analogRead(DISTANCEPIN)-MINLENGTH; //read distance, adjusted for defined minimum length to set zero point. TODO adapt to laser sensor.
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
  Serial.println("Stopactuator, all valves set -10 from startpos. Script delay 1s");
  eservofill.write(SERVOSTARTPOS-10); 
  eservoflush.write(SERVOSTARTPOS-10);
  rservofill.write(SERVOSTARTPOS-10); 
  rservoflush.write(SERVOSTARTPOS-10);
  delay(1000); //to avoid serial spam
}

void bleedactuator() { //slowly release pressure based on input (for shutdown)
  Serial.println("Bleedactuator, input valves set -10 from startpos and flush open. Script delay 1s");
  eservofill.write(SERVOSTARTPOS-10); 
  eservoflush.write(SERVOSTARTPOS+BLEEDANGLE);
  rservofill.write(SERVOSTARTPOS-10); 
  rservoflush.write(SERVOSTARTPOS+BLEEDANGLE);
  delay(1000); //to avoid serial spam
}

/*
  SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent() {
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
    if (movespeed < 0) {wantpos=0;} //asked for negative speed
  }
}
