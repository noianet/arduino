/*
  Simple routine for maual servo adjust (serial)
  Input serial: servoID,angle
  Example: 1,120 - servo 1 to angle 120
*/
#include <Servo.h>
#define LOG_PERIOD 1000  //Serial output logging period in milliseconds.

//PIN declarations:
#define SERVO1 6
#define SERVO2 7
#define SERVO3 8
#define SERVO4 9
#define SERVOSTARTPOS 90 //start angle

//servo declaratons for valve control
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//define and set some global variables
unsigned long previousMillis;  //variable for time measurement
unsigned int counter = 0; //benchmark

void setup()
{
  Serial.begin(115200);

  servo1.attach(SERVO1);
  servo2.attach(SERVO2);
  servo3.attach(SERVO3);
  servo4.attach(SERVO4);
  servo1.write(SERVOSTARTPOS);
  servo2.write(SERVOSTARTPOS);
  servo3.write(SERVOSTARTPOS);
  servo4.write(SERVOSTARTPOS);

  //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW);
  Serial.println("Arduino setup complete");
}

void loop() {
  unsigned long currentMillis = millis(); //time concept

  //Serial output every LOG_PERIOD milliseconds
  if (currentMillis - previousMillis > LOG_PERIOD) {
    previousMillis = currentMillis;
    //Serial.print("loops/sec:: ");
    //Serial.print(counter);
    Serial.print(" srv1: ");
    Serial.print(servo1.read());
    Serial.print(" srv2: ");
    Serial.print(servo2.read());
    Serial.print(" srv3: ");
    Serial.print(servo3.read());
    Serial.print(" srv4: ");
    Serial.println(servo4.read());
    //counter = 0;
  }
  //counter++;
}

void serialEvent() { //runs on interrupt hardware RX.
  while (Serial.available() > 0) {
    // look for the next valid integer in the incoming serial stream:
    byte servoid = Serial.parseInt();
    int servoangle = Serial.parseInt();
    //Echo received variables back to master for verification, which must then retransmit if error
    if (Serial.read() == '\n') {
      Serial.print(servoid);
      Serial.print(",");
      Serial.println(servoangle);
     if (servoid == 1) {servo1.write(servoangle);}
     if (servoid==2) {servo2.write(servoangle);}
     if (servoid==3) {servo3.write(servoangle);}
     if (servoid==4) {servo4.write(servoangle);}
    }
  }
}
