#include <I2Cdev.h>
#include <MPL3115A2.h>

MPL3115A2 mpl;

// Does not work, System Voltage too high?

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  if (!mpl.testConnection()) {
    Serial.println("connection failed\n");
    while(1);
  }
}

void loop() {
  Serial.println(mpl.getAltitude());
  delay(500);
}
