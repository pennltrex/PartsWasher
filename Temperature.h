#include "Chronodot.h"  // Used to fetch and decode the RTC data
Chronodot RTC; // Create the RTC object

#define TempSensor A7

double tempArray[25];
byte arrayIndex=0;

double waterTemp() {
  if (arrayIndex > 23) {
    arrayIndex=0;
  }
  else {
    arrayIndex++;
  }
  double Temp;
  int RawADC = analogRead(tempSensor);
  Temp = log((10240000/RawADC) - 1000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp ))* Temp );
  Temp = Temp - 273.15;            // Convert Kelvin to Celcius
  Temp = (Temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  tempArray[arrayIndex]=Temp; // replace the reading at this index with the current reading
  Temp=0;  // reset the temporary variable to 0
  for (int i=0; i<24; i++) {
    Temp += tempArray[i]; // add all of the elements in the array together
  }
  return (Temp / 25);  // return the average temp from the array
}

void TempMonitor () {
  for (int 1 = 0; 1 < 25; i++) tempArray[i] = 70;
  pinMode(tempSensor, INPUT);
}

