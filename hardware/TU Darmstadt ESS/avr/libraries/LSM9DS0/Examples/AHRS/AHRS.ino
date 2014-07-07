#include <ahrs.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <LSM9DS0.h>

LSM9DS0 sen;
boolean connection=false;
unsigned long time=0;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // neccesary! otherwise no sensor will work
  delay(1000);

  sen.initialize();
  connection = sen.testConnection();
  Serial.println(  connection  ? "OK" : "FAILED");

  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_190);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_HIGH);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);

  sen.setAccRate(LSM9DS0_ACC_RATE_200);
  sen.setAccFullScale(LSM9DS0_ACC_8G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_50);

  sen.setMagFullScale(LSM9DS0_MAG_4_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_100);
}

#define p(x) Serial.print(x)

void loop() {
  if (!connection)
    return;

   time = micros();
   measurement_t m = sen.getMeasurement();
   orientation_t *o = 
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, m.mz,
                  (micros() - time)/1e6);

   p(o->q0); p("\t");
   p(o->q1); p("\t");
   p(o->q2); p("\t");
   p(o->q3); p("\n");
}
