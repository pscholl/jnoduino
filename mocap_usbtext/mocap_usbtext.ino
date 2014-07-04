#include <ahrs.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <LSM9DS0.h>

LSM9DS0 sen;
boolean connection=false;
unsigned long time=0;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // wait until somebody is there

  sen.initialize();
  connection = sen.testConnection();
  Serial.println(  connection  ? "OK" : "FAILED");

  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_HIGH);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);
  sen.setFIFOEnabled(false);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_362);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_50);
}

#define p(x) Serial.print(x)

void loop() {
  if (!connection)
    return;

   time = micros();
   measurement_t m = sen.getMeasurement();
   time = micros() - time;
   
   orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, m.mz,
                  time/1e6);
                  
   if (isnan(o->q0)) { // let's reset
     o->q0 = 1;
     o->q1 = o->q2 = o->q3 = 0;
     o->exInt = o->eyInt = o->ezInt = 0;

//     p(m.gx);p("\t");p(m.gy);p("\t");p(m.gz);p("\t");
//     p(m.ax);p("\t");p(m.ay);p("\t");p(m.az);p("\t");
//     p(m.mx);p("\t");p(m.my);p("\t");p(m.mz);p("\n");

     return;
   }
   
   p(o->q0); p("\t");
   p(o->q1); p("\t");
   p(o->q2); p("\t");
   p(o->q3); p("\n");
}
