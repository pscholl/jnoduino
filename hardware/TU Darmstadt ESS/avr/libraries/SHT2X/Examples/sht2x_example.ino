#include <Wire.h>
#include <I2Cdev.h>
#include <SHT2X.h>

SHT2X sht;
boolean connection=false;

void setup() {
  Wire.begin();
  Serial.print("testing connection: ");
  connection = sht.initialize();
  Serial.println(  connection  ? "OK" : "FAILED");
}

void loop() {
  if (!connection)
    return;
    
  Serial.print("relative humidity (%RH): ");  
  Serial.print(sht.getRelativeHumidity());
  Serial.print("\t");
  Serial.print("temperature (°C): "); 
  Serial.println(sht.getTemperature());
}
