/* 
  pneumatic testjig with statemachine. 
  
  Remember to short emergency pin to ground to run actuator (internal pullup)
*/
#include <Servo.h>
#define LOG_PERIOD 1000  //Serial output logging period in milliseconds.

//PIN declarations:
#define EMERGENCYPIN 3 //internal pullup enabled
#define ESERVOPINFILL 6 //extend servo1
#define ESERVOPINFLUSH 7 //extend servo1 
#define RSERVOPINFILL 8 //retract servo1
#define RSERVOPINFLUSH 9 //retract servo1 
#define DISTANCEPIN A0 //distance sensor
#define EPNEUMATICPIN A1 //pneumatic sensor extend line
#define RPNEUMATICPIN A2 //pneumatic sensor retract line
#define POTMETERPIN A6 //for manual position adjust

//Physical definitions:
#define MAXLENGTH 1000 //max stroke distance in mm
#define MINLENGTH 100 //min stroke distance in mm
#define SERVOSTARTPOS 30 //zero position for valve servos. Physical adjust almost open in this position.NB: emergency set this at -10
#define SERVORANGE 130 //max movement above startpos (added from startpos)
#define MINIMUMPRESSURE 100 //ideal working pressure. This factor will control energy efficiency (too much or too little venting) and somewhat speed,
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

void setup()
{
  Serial.begin(SERIALSPEED);
  
  //attach servos and set to closed position
  eservofill.attach(ESERVOPINFILL); 
  eservoflush.attach(ESERVOPINFLUSH); 
  rservofill.attach(RSERVOPINFILL); 
  rservoflush.attach(RSERVOPINFLUSH); 
  stopactuator(); //sets all servos to closed position

  //enable pullup for emergency read. Short to ground to run actuator
  pinMode(EMERGENCYPIN, INPUT_PULLUP);
  
  //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW); 
  
  Serial.println("Arduino setup complete");
}

void loop() 
{ 
  //read potmeter, translate to want position (adjust speed via serial - obs for required wantpos)
  wantpos = map(analogRead(POTMETERPIN), 0, 1023, MINLENGTH, MAXLENGTH); // scale to stroke length
  
  //check if emergency, pull to ground for actuator run (internal pullup enabled)
 if (!digitalRead(EMERGENCYPIN)) { //if emergency pin NOT shorted to ground run stopactuator. Pin has enabled internal pullup.
    stopactuator(); //close all valves
  } else {
    runactuator(); //run PID statemachine.
  }
  
  
  //Serial output every LOG_PERIOD milliseconds
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    Serial.print("DEBUG: speed: ");
    Serial.print(movespeed);
    Serial.print(" epresssure: ");
    Serial.print(extendpressure);
    Serial.print(" rpresssure: ");
    Serial.print(retractpressure);
    Serial.print(" wantpos: ");
    Serial.println(wantpos);
    Serial.print(" havepos: ");
    Serial.println(havepos);
  }
} 
//----------------------------------------END INIT/SETUP/LOOP, utility functions below-----------------------------------------------------

void runactuator() { //simple statemachine, operates by calls to position reads and global vars for wants/speeds
  //sanity check for movespeed input, do not go beyond defined max open. (will happen if movespeed is set too high)
  if (movespeed > SERVORANGE) {movespeed=SERVORANGE;}

  //read some sensor values
  havepos=analogRead(DISTANCEPIN); //read distance. TODO adapt to sensor. potmeter for test
  extendpressure=analogRead(EPNEUMATICPIN);
  retractpressure=analogRead(RPNEUMATICPIN);

  // Try to hold position and a MINIMUMPRESSURE (to not vent down to atmosphere). 
  // if position requires pressure below MINIMUMPRESSURE, hold valves closed (opposite direction will compensate)
  
  if ((havepos+HYSTERISIS) < wantpos) { //need to extend
    rservofill.write(SERVOSTARTPOS); //close input on retract line
    eservoflush.write(SERVOSTARTPOS); //close flush on extend line
    eservofill.write(SERVOSTARTPOS+movespeed); //open fill on extend line
    if (retractpressure>MINIMUMPRESSURE) { //check if above minimum pressure on retract line, eject air if is, else close
      rservoflush.write(SERVOSTARTPOS+movespeed);   
     } else {
      rservoflush.write(SERVOSTARTPOS); 
     }  
  } else if ((havepos-HYSTERISIS) > wantpos) { //need to retract
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
  Serial.println("DEBUG: Stopactuator, all valves set -10 from startpos");
  eservofill.write(SERVOSTARTPOS-10); 
  eservoflush.write(SERVOSTARTPOS-10);
  rservofill.write(SERVOSTARTPOS-10); 
  rservoflush.write(SERVOSTARTPOS-10);
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
  }
}
