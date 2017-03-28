/* 
 3dof servo testjig. 
   
   Assignations and global variables:
     lservo = left back
       lwantpos = want position
       lspeed = want speed to achieve
       lhavepos = current position reading
     rservo = right back
       rwantpos = want position
       rspeed = want speed to achieve
       rhavepos = current position reading
     fservo = front,
       rwantpos = want position
       rspeed = want speed to achieve   
       rhavepos = current position reading

   Utility pins:
     Analog read potpin for manual adjustments <------------removed
     Digital read, pullup for emergency stop (close all valves)
     
   General USB I/O, commIO():
     Serial out for "current positions" - fakery for servo mode
     Serial in wanted position and speeds. direct translation to global vars:
        lwantpos,lspeed,rwantpos,rspeed,fwantpos,rspeed  (newline)
        ex: 60,3,60,4,60,5
       MaxSpeed = 40 (above is set=40, only delay adjust in runactuator function for now)
   
   Functions:
     demo() : position generator "wants" to do stuff. generates values instead of serial in gcode reading. Temporary, TODO: move to master controller.
     commIO() serial write curernt position and serialread to global var translation. rewrite for I2C etc
     readposition(); read current positions, utility called from others
     stopactuator(); emergency mode, close all vaves. 
     runactuator(): no input, all IO variables used are global. <- complecity/PID statemachine in here. 
*/
//PIN and misc declarations:
//#define POTMETERPIN A1 //analog read
#define EMERGENCYPIN 2 //internal pullup enabled, boolean
#define LSERVOPIN 9 //left servo 
#define RSERVOPIN 5 //right servo 
#define FSERVOPIN 7 //front servo 
#define SERIALSPEED 115200
#define DELAYLOOP 1 //delay at end in loop
//TODO: add defines LREADPIN etc. for distance sensors, used in TODO in readposition.

#include <Servo.h>

//servo declaratons
Servo lservo;  // left back servo
Servo rservo;  // right back servo
Servo fservo;  // front back servo

//utility global vars
//int potval=0; //potentiometer
boolean emergency = false; //emergency button read

//set some initial global variables with median want pos and (slow) speeds. havepos read in runactuator
int lwantpos=10;
int lspeed=2;
int lhavepos; 
int rwantpos=70;
int rspeed=2;
int rhavepos; 
int fwantpos=130;
int fspeed=2;
int fhavepos; 

void setup()
{
  Serial.begin(SERIALSPEED);
  
  lservo.attach(LSERVOPIN);  //set PWM pin
  rservo.attach(RSERVOPIN);  //set PWM pin
  fservo.attach(FSERVOPIN);  //set PWM pin

  //enable pullup for emergency read. Add resistor to ground for operation.
  pinMode(EMERGENCYPIN, INPUT_PULLUP);
  
  //TODO LAST: pull all unused pins to output low for noise suppression. Should be resistor pulldown in HW as well.
  //pinMode(3, OUTPUT); digitalWrite(3, LOW);
  
  //TODO: add slow move to initial position/calibration loop
  
  //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW); 
  
  Serial.println("Arduino setup complete");
}

void loop() 
{ 
  //check if emergency, resistor to ground for enable (internal pullup enabled
  //emergency = digitalRead(EMERGENCYPIN);
  //TODO: remove comment and pull low with resistor to run
  
  //read pot-pin for manual adjust
  //potval = analogRead(POTMETERPIN);            // reads the value of the potentiometer (value between 0 and 1023) 
  //potval = map(potval, 0, 1023, 0, 90);     // scale it to use it with the servo (value between 0 and 180) 
  
 if (emergency) {
    stopactuator(); //close all valves
  } else {
    runactuator(); //run PID statemachine.
  }
  
  demo(); //simple value generator, make some moves
  commIO(); //communicate with master. send current position and read new instructions.
  delay(DELAYLOOP);
} 
//----------------------------------------END INIT/SETUP/LOOP, utility functions below-----------------------------------------------------

void demo() { //temporary, move to master controller
  //catch if have met maximum and reset
  //increase position based on wanted speed;
  lwantpos=lwantpos+lspeed;
  rwantpos=rwantpos+rspeed; 
  fwantpos =fwantpos+fspeed;

  
  if (lwantpos > 180) {
    lwantpos=0;
  } 
    if (rwantpos > 180) {
    rwantpos=0;
  } 
    if (fwantpos > 180) {
    fwantpos=0;
  } 
  //increase position
  //lwantpos++;  
  //rwantpos++;  
  //fwantpos++;  
  
  delay(40);
}

void commIO() {
   //report position data to out
  Serial.print("lpos: ");
  Serial.print(lhavepos);
  Serial.print(" rpos: ");
  Serial.print(rhavepos);
  Serial.print(" fpos: ");
  Serial.println(fhavepos);
  
  //listen to serial for integer values. fotmat is:
  //lwantpos,lspeed,rwantpos,rspeed,fwantpos,rspeed 
  //ex: 60,20,60,20,60,20
  ///remember newline at end
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    lwantpos = Serial.parseInt();
    lspeed = Serial.parseInt();
    rwantpos = Serial.parseInt();
    rspeed = Serial.parseInt();
    fwantpos = Serial.parseInt();
    fspeed = Serial.parseInt();
    //Response is echo variables back to master for verification, which must then retransmit if error
    if (Serial.read() == '\n') {
      Serial.print(lwantpos);
      Serial.print(",");
      Serial.print(lspeed);
      Serial.print(",");
      Serial.print(rwantpos);
      Serial.print(",");
      Serial.print(rspeed);
      Serial.print(",");
      Serial.print(fwantpos);
      Serial.print(",");
      Serial.println(fspeed);
      //delay(1000);
    }
  }
}

void stopactuator() {
  //TODO: setclose all vaves. Servo test=do nothing/delay
  Serial.println("DEBUG: Stopactuator");
  delay(15);  
}

void runactuator() { //PID statemachine, operates by calls to position reads and global vars for wants/speeds
  //just populate have pos=want pos.
  lhavepos=lwantpos;
  rhavepos=rwantpos;
  fhavepos=fwantpos;
  //TODO: add some fudge/fakery/fiction
  
  //set some positions
  lservo.write(lwantpos);  // sets the servo position 
  rservo.write(rwantpos);  // sets the servo position
  fservo.write(fwantpos);  // sets the servo position
  
  //uses only lspeed to adjust, sanitycheck for value.
  //if (lspeed < 40) {
  //  delay(40-lspeed);
  //} 
}
