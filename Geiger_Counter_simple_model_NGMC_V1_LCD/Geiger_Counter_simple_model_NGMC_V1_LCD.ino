/*
Geiger counter details:
  Geiger interrupt 0 (pin D2 on arduino mini - see attachInterrupt). 
  Onboard led swap on/off on geiger read (13 on mini)
  Serial output 115200. Indicates CPM, uSv og uSv average (floating average, 4 readings)
  LCD support (pin 3, 4, 5, 6, 7, 8)
  External 5 LED row indicator support (pin A0, A1 A2, A3, A4) - analog pins for tidyness since not used elsewhere,
  
  TODO: Store to large register and serial input code for dump.
*/

#include <LiquidCrystal.h>
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(3,4,5,6,7,8);

#define LOG_PERIOD 15000  //Logging period in milliseconds, recommended value 15000-60000.
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

void setup(){  
  counts = 0;
  cpm = 0;
  multiplier = MAX_PERIOD / LOG_PERIOD;      //calculating multiplier, depend on your log period
  Serial.begin(115200);
  attachInterrupt(0, tube_impulse, FALLING);   //define external interrupt 0
  
  //set up the LCD\'s number of columns and rows:
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Radiation Sensor");
  lcd.setCursor(0,1);
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
    usv_average=((usv_average*3+usv)/4);
    
    lcd.clear();    
    lcd.setCursor(0, 0);
    lcd.print("CPM:");
    lcd.setCursor(4,0);    //lcd.setCursor(9, 0);
    lcd.print(cpm);
    
    lcd.setCursor(8, 0);
    lcd.print((char)228); //special char µ
    lcd.print("Sv:");
    lcd.setCursor(12,0);
    lcd.print(usv,2);
    
    lcd.setCursor(0,1);
    lcd.print(usv_average,4);
    lcd.setCursor(7,1);
    lcd.print((char)228); //special char µ
    lcd.print("Sv/h avg");
    
    //led array 
    if(cpm <= TH1) ledVar(0);
    if((cpm <= TH2)&&(cpm>TH1)) ledVar(1);
    if((cpm <= TH3)&&(cpm>TH2)) ledVar(2);
    if((cpm <= TH4)&&(cpm>TH3)) ledVar(3);
    if((cpm <= TH5)&&(cpm>TH4)) ledVar(4);
    if(cpm>TH5) ledVar(5);
    
    Serial.print("cpm: ");
    Serial.print(cpm);
    Serial.print(" uSv/h: ");
    Serial.print(usv);
    Serial.print(" uSv/h avg: ");
    Serial.println(usv_average);
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

void tube_impulse(){       //function for capturing interrupt events from Geiger Kit
  counts++;
  //switch state onboard led to indicate response
  digitalWrite(13, !digitalRead(13));
}


