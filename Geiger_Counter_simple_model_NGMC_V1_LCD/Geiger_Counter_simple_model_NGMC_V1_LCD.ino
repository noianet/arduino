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
  LCD support I2C, A4=SDA, A5=SCL.
  External 5 LED row indicator support with fade for increased resolution. Indicates counts pr. 10/sec.
  
  TODO: Store to SD card?
  TODO: Buttons to do some stuff?
  TODO: rad accumulator calculation and a "run away" warning?
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x3F,20,4);
SoftwareSerial btSerial(11, 12); // RX, TX for bluetooth adapter

#define LOG_PERIOD 30000  //Logging period in milliseconds, recommended value 15000-60000. NB: correct usv_accumulated calculation if changed
#define MAX_PERIOD 60000  //Maximum logging period without modifying this sketch

#define USV_CONVERSION 123.147092360319  //conversion factor for J305 tube. Factor: 0.00812037037037
  //link to data for J305: https://www.cooking-hacks.com/documentation/tutorials/geiger-counter-radiation-sensor-board-arduino-raspberry-pi-tutorial/
//#define USV_CONVERSION 151.5 //for M4011 tube, 
//#define USV_CONVERSION 175.43 //for SBM-20 tube

// CPS threshold values for the led bar (resets 10 times pr sec),
#define TH0 40 
#define TH1 80 
#define TH2 120 
#define TH3 160 
#define TH4 200

//Pins used for LED array. Must be PWM capable
int ledArray [] = {3,5,6,9,10};

// Variables
unsigned long counts=0;     //variable for GM Tube events
unsigned long cpm=0;        //variable for CPM
unsigned long ledcps=0;        //led counts pr. 10/second
unsigned int multiplier;  //variable for calculation CPM in this sketch
unsigned long previousMillis;  //variable for time measurement
unsigned long previousLedMillis;  //variable for time measurement
float usv_average=0.20;  //variable for uSv, starting with avg. 0.20
float usv_average_old=0.20;  //variable for uSv last reading for LCD arrow, starting with avg. 0.20
float usv_accumulated=0;  //variable.for accumulated since boot/reset
boolean lcd_mode=1; //used to swap LCD info

void setup(){  
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
  Serial.begin(115200);
  btSerial.begin(9600);
  attachInterrupt(0, tube_impulse, FALLING);   //define external interrupt 0
  
  ledFade(TH4-1); //all on LEDBAR
  //set up the LCD\'s number of columns and rows:
  lcd.init();                      // initialize the lcd 
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Radiation Sensor");
  lcd.setCursor(2,1);
  lcd.print("Please wait"); 


   //flash onboard led to indicate setup done
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1500);
  ledFade(TH2-1);  //leddbar medium
  digitalWrite(13, LOW); 
  delay(1000);
  
  ledFade(0); //reset ledbar
  
  Serial.println("Arduino setup complete");
}

void loop(){                                
  unsigned long currentMillis = millis();
  
  //Time sequence for LCD and serial updates
  if(currentMillis - previousMillis > LOG_PERIOD){
    previousMillis = currentMillis;
    cpm = counts * multiplier; //adjustable by constants
    
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
  
  if(currentMillis - previousLedMillis > 100){ //run 10 times pr second
    previousLedMillis = currentMillis;
    ledFade(ledcps);
    ledcps=0; //reset counter
  }  
}

void ledFade(int value){  //function for 5x LED row update with fading effect, d for delay which defaults to 0
    
    //Set LED values, mapped from value. In theory should adjust to ranges....
   if(value < TH0) {
         //DEBUG: Print current value
    //Serial.print(" LedCPdS: : ");
    //Serial.println(map(value,0,TH0,0,255));
      analogWrite(ledArray[0],map(value,0,TH0,0,255)); 
      digitalWrite(ledArray[1], LOW); digitalWrite(ledArray[2], LOW); digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if(value < TH1) {
      digitalWrite(ledArray[0], HIGH);
      value=value-TH0;
      analogWrite(ledArray[1],map(value,0,TH1,0,255)); 
      digitalWrite(ledArray[2], LOW); digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if(value < TH2) {
      digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH);
      value=value-TH1;
      analogWrite(ledArray[2],map(value,0,TH2,0,255)); 
      digitalWrite(ledArray[3], LOW); digitalWrite(ledArray[4], LOW);
    } else if(value < TH3) {
      digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH);
      value=value-TH2;
      analogWrite(ledArray[3],map(value,0,TH3,0,255)); 
      digitalWrite(ledArray[4], LOW);
   } else if(value < TH4) {
      digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH); digitalWrite(ledArray[3], HIGH);
      value=value-TH3;
      analogWrite(ledArray[4],map(value,0,TH4,0,255)); 
    } else {
      digitalWrite(ledArray[0], HIGH); digitalWrite(ledArray[1], HIGH); digitalWrite(ledArray[2], HIGH); digitalWrite(ledArray[3], HIGH); digitalWrite(ledArray[4], HIGH);
    }      
}

void tube_impulse(){       //function for capturing interrupt events from Geiger Kit
  counts++;
  ledcps++;
  //switch state onboard led to indicate response
  digitalWrite(13, !digitalRead(13));
}


