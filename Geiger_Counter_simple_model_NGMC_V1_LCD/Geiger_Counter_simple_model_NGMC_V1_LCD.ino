/*
GPL ref, based on https://github.com/majek/dump/blob/master/arduino/ard-01/geiger.pde

Geiger counter details:
  Geiger interrupt 0 (pin D2 on arduino mini - see attachInterrupt). 
  Onboard led swap on/off on geiger read (13 on mini)
  Serial USB output 115200. Indicates CPM, µSv og µSv average (floating average over 10 readings)
     Example output: cpm: 20 uSv/h: 0.16 uSv/h avg: 0.17
  Softserial for bluetooth mobile reporting. Adapted for app  "Bluetooth Terminal/Graphics" by Emrecan ÇETİN. Uses pin 11=RX, 12=TX (RX not in use for now)
     Format required by app:  Evalue1,value2,value3...\n. Data sent are: Ecpm,usv,usv_average\n
     Using HC-06 bt device, usually pairing passcode is 1234.
  LCD support (pin 3,4,5,6,7,8)
  External 5 LED row indicator support (pin A0,A1,A2,A3,A4). Separate funtion for led fade effect.
  
  TODO: Store CPM to large register/stack and serial input command for datadump. Include some LCD display update "download mode" for example).
  TODO: Buttons to do some stuff?
  TODO: rad accumulator calculation and a "run away" warning?
*/
#include <LiquidCrystal.h>
#include <SoftwareSerial.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3,4,5,6,7,8);
SoftwareSerial btSerial(11, 12); // RX, TX

#define LOG_PERIOD 30000  //Logging period in milliseconds, recommended value 15000-60000. NB: correct usv_accumulated calculation if changed
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch

#define USV_CONVERSION 123.147092360319  //conversion factor for J305 tube. Factor: 0.00812037037037
  //link to data for J305: https://www.cooking-hacks.com/documentation/tutorials/geiger-counter-radiation-sensor-board-arduino-raspberry-pi-tutorial/
//#define USV_CONVERSION 151.5 //for M4011 tube, 
//#define USV_CONVERSION 175.43 //for SBM-20 tube

// CPM threshold values for the led bar
#define TH1 45
#define TH2 95
#define TH3 200
#define TH4 400
#define TH5 600

//Pins used for LED array (digital writes)
int ledArray [] = {A0,A1,A2,A3,A4};

// Variables
unsigned long counts;     //variable for GM Tube events
unsigned long cpm;        //variable for CPM
unsigned int multiplier;  //variable for calculation CPM in this sketch
unsigned long previousMillis;  //variable for time measurement
float usv_average=0.20;  //variable for uSv, starting with avg. 0.20
float usv_average_old=0.20;  //variable for uSv last reading for LCD arrow, starting with avg. 0.20
float usv_accumulated=0;  //variable.for accumulated since boot/reset
boolean lcd_mode=1; //used to swap LCD info

void setup(){  
  counts = 0;
  cpm = 0;
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
  Serial.begin(115200);
  btSerial.begin(9600);
  attachInterrupt(0, tube_impulse, FALLING);   //define external interrupt 0
  
  //set up the LCD\'s number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Radiation Sensor");
  lcd.setCursor(2,1);
  lcd.print("Please wait"); 

   //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  digitalWrite(13, LOW); 
  delay(1000);
  
  Serial.println("Arduino setup complete");
}

void loop(){                                
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    cpm = counts * multiplier;
    
    //const float conversion_factor
    float usv = (float)cpm / USV_CONVERSION;
    usv_average=((usv_average*9+usv)/10);
    usv_accumulated=usv_accumulated+(usv/2)/60;//accumulated since boot/reset
    
    lcd.clear();    
    lcd.setCursor(0, 0);
    if (lcd_mode) { //swaps between two LCD infos on first line
      if (cpm < 10){lcd.print(" ");} //add spaces to avoid jumping on 1, 10 digit numbers
      if (cpm < 100){lcd.print(" ");}
      if (cpm < 1000){lcd.print(" ");}
      lcd.print(cpm);
      lcd.print("cpm");
      lcd.setCursor(8, 0);
      lcd.print(usv,2);
      lcd.print((char)228); //special char µ
      lcd.print("Sv");    
      lcd_mode=0;
    } else {
      lcd.print(" ");    
      lcd.print((char)228); //special char µ
      lcd.print("Sv acc: ");
      lcd.print(usv_accumulated);
      lcd_mode=1;
    }
    lcd.setCursor(0,1); //second line
    if (usv_average > usv_average_old) {lcd.print((char)126);} //special char, right arrow (rom missing up)
    else if (usv_average < usv_average_old) {lcd.print((char)127);} //special char, left arrow (rom missing down)
    else {lcd.print(" ");} //no change, space only
    lcd.print(usv_average,3);
    lcd.print((char)228); //special char µ
    lcd.print("Sv/h avg");
    usv_average_old=usv_average; //update with last reading
    
    //led array 
    if(cpm <= TH1) ledVar(0);
    if((cpm <= TH2)&&(cpm>TH1)) ledVar(1);
    if((cpm <= TH3)&&(cpm>TH2)) ledVar(2);
    if((cpm <= TH4)&&(cpm>TH3)) ledVar(3);
    if((cpm <= TH5)&&(cpm>TH4)) ledVar(4);
    if(cpm>TH5) ledVar(5);
    
    //USB serial
    Serial.print("cpm: ");
    if (cpm < 10) {Serial.print(" ");} //adds extra space if single digit to clean up formatting
    //if (cpm < 100) {Serial.print(" ");} //adds extra space if single digit to clean up formatting
    //if (cpm < 1000) {Serial.print(" ");} //adds extra space if single digit to clean up formatting
    Serial.print(cpm);
    Serial.print(" uSv/h: ");
    Serial.print(usv);
    Serial.print(" uSv/h avg: ");
    Serial.print(usv_average);
    Serial.print(" uSv/h acc: ");
    Serial.println(usv_accumulated,4);
    
    //Bluetooth softserial 
    btSerial.print("E");
    btSerial.print(cpm);
    btSerial.print(",");
    btSerial.print(usv);
    btSerial.print(",");
    btSerial.print(usv_average);
    btSerial.print("\n");
    
    counts = 0;
  }
}

void ledVar(int value){  //function for 5x LED row update
  if (value > 0){
    for(int i=0;i<=value;i++){
      digitalWrite(ledArray[i],HIGH);
    }
    for(int i=5;i>value;i--){
      digitalWrite(ledArray[i],LOW);
    }
  }
  else {
    for(int i=5;i>=0;i--){
      digitalWrite(ledArray[i],LOW);
    }
  }
}

void ledFade(int value, int d = 0){  //function for 5x LED row update with fading effect, d for delay which defaults to 0
  if (value > 0){
    for(int i=0;i<=value;i++){
      // fade in from min to max in increments of 5 points:
      for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
        // sets the value (range from 0 to 255):
        analogWrite(ledArray[i], fadeValue);
        // wait for d milliseconds to see the dimming effect
       delay(d);
      }
    }
    for(int i=5;i>value;i--){
      // fade out from max to min in increments of 5 points:
      for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
        // sets the value (range from 0 to 255):
        analogWrite(ledArray[i], fadeValue);
        // wait for d milliseconds to see the dimming effect
        delay(d);
      }
    }
  } 
  else {
    for(int i=5;i>=0;i--){
      // fade out from max to min in increments of 5 points:
      for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
        // sets the value (range from 0 to 255):
        analogWrite(ledArray[i], fadeValue);
        // wait for d milliseconds to see the dimming effect
        delay(d);
      }
    }
  }
}

void tube_impulse(){       //function for capturing interrupt events from Geiger Kit
  counts++;
  //switch state onboard led to indicate response
  digitalWrite(13, !digitalRead(13));
}


