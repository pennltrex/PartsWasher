#include <Arduino.h>

/********************
Arduino generic menu system
Arduino menu on I2C LCD example
http://www.r-site.net/?at=//op%5B%40id=%273090%27%5D

Sep.2014 Rui Azevedo - ruihfazevedo(@rrob@)gmail.com

LCD library:
https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home
http://playground.arduino.cc/Code/LCD3wires
*/
#ifndef ARDUINO_SAM_DUE

  #include <Wire.h>
  #include <LiquidCrystal_I2C.h>//F. Malpartida LCD's driver
  #include <menu.h>//menu macros and objects
  #include <menuIO/lcdOut.h>//malpartidas lcd menu output
  #include <menuIO/serialIn.h>//Serial input
  #include <menuIO/encoderIn.h>//quadrature encoder driver and fake stream
  #include <menuIO/keyIn.h>//keyboard driver and fake stream (for the encoder button)
  #include <menuIO/chainStream.h>// concatenate multiple input streams (this allows adding a button to the encoder)

  using namespace Menu;

  LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

  // Setup Rotary Encoder
  #define encA 10
  #define encB 11
  #define encBtn 8

  encoderIn<encA,encB> encoder;
  #define ENC_SENSIVITY 4
  encoderInStream<encA,encB> encStream(encoder,ENC_SENSIVITY);

  keyMap encBtn_map[]={{-encBtn,defaultNavCodes[enterCmd].ch}};
  keyIn<1> encButton(encBtn_map);

  serialIn serial(Serial);
  menuIn* inputsList[]={&encStream,&encButton,&serial};
  chainStream<3> in(inputsList);//3 is the number of inputs
  // End rotary encoder

  #define LEDPIN A3
  
  
  #define RELAY1 6 //WaterIn
  #define RELAY2 5 //WashMotor
  #define RELAY3 4 //Heat
  #define RELAY4 3 //WaterOut

  // Temp Sensor

  // which analog pin to connect
#define THERMISTORPIN A0         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 9880      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 10
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000    

int samples[NUMSAMPLES];

double temp() {
  uint8_t i;
  float average;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
  
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  Serial.print("Average analog reading "); 
  Serial.println(average);
  
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
  
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert absolute temp to C
// steinhart = (steinhart * 9/5) + 32;        // conver C to F
  
}
// Initials states for the relays
  int ledCtrl=LOW;

  int WaterIn=HIGH;
  int WashMotor=HIGH;
  int Heater=HIGH;
  int WaterOut=HIGH;


  result fillCycle() {
    WaterIn=LOW;
    WashMotor=HIGH;
    Heater=HIGH;
    WaterOut=HIGH;
    ledCtrl=HIGH;
    return proceed;
  }
  
  result heatCycle() {
    WaterIn=HIGH;
    WashMotor=HIGH;
    Heater=LOW;
    WaterOut=HIGH;
    return proceed;
  }
  
  result runCycle() {
    WaterIn=HIGH;
    WashMotor=LOW;
    while (temp() <= 65)
    {
      Heater=HIGH;
    }
    WaterOut=HIGH;
    return proceed;
  }
  
  result drainCycle() {
    WaterIn=HIGH;
    WashMotor=HIGH;
    Heater=HIGH;
    WaterOut=LOW;
    return proceed;
  }
  
  result offCycle() {
    WaterIn=HIGH;
    WashMotor=HIGH;
    Heater=HIGH;
    WaterOut=HIGH;
    ledCtrl=LOW;
    return proceed;
  }


// MENU(mainMenu, "Blink menu", Menu::doNothing, Menu::noEvent, Menu::wrapStyle
//   ,FIELD(timeOn,"On","ms",0,1000,10,1, Menu::doNothing, Menu::noEvent, Menu::noStyle)
//   ,FIELD(timeOff,"Off","ms",0,10000,10,1,Menu::doNothing, Menu::noEvent, Menu::noStyle)
//   ,EXIT("<Back")
// );

  MENU(mainMenu, "Main Menu", doNothing, noEvent,wrapStyle
    ,OP("Fill", fillCycle,enterEvent)
    ,OP("Nothing", doNothing,noEvent) // This one just displays a - cursor instead of a > and if I add a function to it, does nothing
    ,OP("Heat", heatCycle,enterEvent)
    ,OP("Run", runCycle,enterEvent)
    ,OP("Drain", drainCycle,enterEvent)
    ,OP("Off",offCycle,enterEvent)
    
  );

  #define MAX_DEPTH 2

 MENU_OUTPUTS(out,MAX_DEPTH
    ,LCD_OUT(lcd,{0,0,20,4})
    ,NONE
  );
    
  NAVROOT(nav,mainMenu,MAX_DEPTH,in,out);//the navigation root object

  result idle(menuOut& o,idleEvent e) {
    switch(e) {
      case idleStart:o.print("suspending menu!");break;
      case idling:o.print("suspended...");break;
      case idleEnd:o.print("resuming menu.");break;
    }
    return proceed;
  }
  
  void setup() {
    digitalWrite(RELAY1, HIGH);
    digitalWrite(RELAY2, HIGH);
    digitalWrite(RELAY3, HIGH);
    digitalWrite(RELAY4, HIGH);

    pinMode(encBtn,INPUT_PULLUP);
    pinMode(LEDPIN,OUTPUT);
    pinMode(RELAY1,OUTPUT);
    pinMode(RELAY2,OUTPUT);
    pinMode(RELAY3,OUTPUT);
    pinMode(RELAY4,OUTPUT);
    Serial.begin(115200);
    while(!Serial);
    Serial.println("Arduino Menu Library");Serial.flush();
    encoder.begin();
    lcd.begin(20,4);
    nav.idleTask=idle;//point a function to be used when menu is suspended
    mainMenu[1].enabled=disabledStatus;
    nav.showTitle=false;
    lcd.setCursor(0, 0);
    lcd.print("Menu 4.x LCD");
    lcd.setCursor(0, 1);
    lcd.print("r-site.net");
    delay(2000);
  }

  void loop() {
    nav.poll();
    digitalWrite(LEDPIN, ledCtrl);
    digitalWrite(RELAY1, WaterIn);
    digitalWrite(RELAY2, WashMotor);
    digitalWrite(RELAY3, Heater);
    digitalWrite(RELAY4, WaterOut);
    //delay(100);//simulate a delay as if other tasks are running
  }

#endif
