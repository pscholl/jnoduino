#include <I2Cdev.h>
#include <LTC2942.h>
#include <Wire.h>

LTC2942 ltc;

#define p(x) Serial.print(x);

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  if (!ltc.testConnection()) {
    Serial.println("connection failed\n");
    while(1);
  }

  p("charge\tvoltage\ttemperature\n");
}

void loop() {
  ltc2942_measurement_t m = ltc.getMeasurement();
  p(m.batteryCharge); p("\t");
  p(m.batteryVoltage); p("\t");
  p(m.temperature); p("\n");
  delay(500);
}
