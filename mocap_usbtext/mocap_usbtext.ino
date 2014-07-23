#include <ahrs.h>
#include <I2Cdev.h>
#include <LSM9DS0.h>
#include <math.h>
#include <EEPROM.h>

#define MAGIC 0x33

LSM9DS0 sen;
float inf = INFINITY;

typedef struct cal {
 float lx, hx, ly, hy, lz, hz;
 float total_max, total_min;
 float offset_x, offset_y, offset_z,
       scale_x, scale_y, scale_z;
} cal_t;

struct cal mag_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };
#define p(x) Serial.print(x)

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // wait until somebody is there

  Serial.println(  sen.testConnection() ? "OK" : "FAILED");

  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_LOW);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);
  //sen.setGyroHighPassMode(LSM9DS0_HPM_NORMAL);
  //sen.setGyroHighPassFilterCutOffFrequencyLevel(LSM9DS0_HPCF10);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_362);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_50);

  if( EEPROM.read(0)-MAGIC == LSM9DS0_MAG_2_GAUSS ) 
  {
    char *p = (char*) &mag_cal;
    for (uint32_t b=0; b<sizeof(struct cal); b++) 
      p[b] = EEPROM.read(b+1);

    p("got calibration data:");
    p(mag_cal.scale_x); p(" ");
    p(mag_cal.scale_y); p(" ");
    p(mag_cal.scale_z); p("    ");

    p(mag_cal.offset_x); p(" ");
    p(mag_cal.offset_y); p(" ");
    p(mag_cal.offset_z); p("\n");
  }
  else
    p("no/wrong calibration data\n");
}

unsigned long time=0;

void loop() {
   time = micros();
   lsm9d_measurement_t m = sen.getMeasurement();
   time = micros() - time;

   m.mx = (m.mx - mag_cal.offset_x) * mag_cal.scale_x;
   m.my = (m.my - mag_cal.offset_y) * mag_cal.scale_y;
   m.mz = (m.mz - mag_cal.offset_z) * mag_cal.scale_z;

   orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, -m.mz,
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

//  p(m.ax); p(F("\t"));
//  p(m.ay); p(F("\t"));
//  p(m.az); p(F("\t"));
//  p(m.mx); p(F("\t"));
//  p(m.my); p(F("\t"));
//  p(m.mz); p(F("\n"));
}
