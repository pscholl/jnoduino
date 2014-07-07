#include <I2Cdev.h>
#include <VCNL4000.h>

VCNL4000 vcnl;

#define p(x) Serial.print(x);

void setup() {
  Serial.begin(115200);

  while (!Serial)
    ;

  if (!vcnl.testConnection()) {
    Serial.println("connection failed\n");
    while(1);
  }

  p("ambient_light\tproximity\n");
}

void loop() {
  p(vcnl.getAmbientLight()); p("\t");
  p(vcnl.getProximity()); p("\n");
  delay(500); // default sampling rate
}
