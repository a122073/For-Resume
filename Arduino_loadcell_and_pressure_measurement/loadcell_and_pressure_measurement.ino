

#include "Wire.h"
#include <SPI.h>
#include <SD.h>
#include <HX711_ADC.h>
#include <EEPROM.h>

const int HX711_dout = 6; 
const int HX711_sck = 7;
const int pressureInput = A0;
float pressureValue = 0;
const int calVal_eepromAdress = 0;
unsigned long t = 0;
const int chipSelect = 4;

HX711_ADC LoadCell(HX711_dout, HX711_sck);

void setup() 


{
  Serial.begin(9600); 
  delay(100);
  Serial.println();
  Serial.println("Starting...");

  LoadCell.begin();
  float calibrationValue;

  
  calibrationValue = -165;


  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue); // set calibration value (float)
    Serial.println("Startup is complete");
  }

{
    Serial.println("CLEARSHEET");
    Serial.println("LABEL,Thrust,Pressure,Timer");
}
  
}
void loop()


{
   
  
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;
  
  if (LoadCell.update()) newDataReady = true;
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) 
    {
      float i = LoadCell.getData();
     
     pressureValue = analogRead(pressureInput);
     pressureValue = (pressureValue-400)/17.5;     

         Serial.print("DATA");
         Serial.print(",");
          Serial.print(LoadCell.getData());
       Serial.print(",");
      
       Serial.print(pressureValue, 1);   
       Serial.print(",");
       Serial.print("TIMER");
       Serial.print(",");
       Serial.println("AUTOSCROLL_20");   
      
      newDataReady = 0;
      t = millis();
 
    }
  }

if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // check if last tare operation is complete:
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }

}
