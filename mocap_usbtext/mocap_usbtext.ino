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
struct cal acc_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };
#define p(x) Serial.print(x)

void setup() {
  Serial.begin(115200);
  while (!Serial)
    ; // wait until somebody is there

  pinMode(A2, OUTPUT);   // turns green LED on
  digitalWrite(A2, LOW); // wire to GND

  sen.initialize();

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

    p = (char*) &acc_cal;
    for (uint32_t b=0; b<sizeof(struct cal); b++) 
      p[b] = EEPROM.read(sizeof(struct cal)+b+2);

    p("got calibration data:");
    p(mag_cal.scale_x); p(" ");
    p(mag_cal.scale_y); p(" ");
    p(mag_cal.scale_z); p("    ");

    p(mag_cal.offset_x); p(" ");
    p(mag_cal.offset_y); p(" ");
    p(mag_cal.offset_z); p("\n");
  }
  else
    p("no calibration data\n");

  pinMode(A2, INPUT); // turns green LED off
}

unsigned long lighton=0, time=0;
static lsm9d_measurement_t m;

void magcal();
void loop() {
   pinMode(A2, lighton=!lighton ? INPUT : OUTPUT);

   time = micros(); sen.updateMeasurement(&m); time = micros() - time;
   m.mz = -m.mz;
   magcal();

   orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, m.mz,
                  1/95.); // half gyro sample rate

   //p(time/1.e6);p("\t");
   p(m.gx);p("\t");p(m.gy);p("\t");p(m.gz);p("\t");
   p(m.mx);p("\t");p(m.my);p("\t");p(m.mz);p("\t");
   p(m.ax);p("\t");p(m.ay);p("\t");p(m.az);
   p("\n");

   if (isnan(o->q0)) { // let's reset
     p ("reset\n");
     o->q0 = 1;
     o->q1 = o->q2 = o->q3 = 0;
     o->exInt = o->eyInt = o->ezInt = 0;
     return;
   }

   p(o->q0); p("\t");
   p(o->q1); p("\t");
   p(o->q2); p("\t");
   p(o->q3);
   p("\n");
}

//static float px=inf,mx=-inf, py=inf,my=-inf, pz=inf,mz=-inf;
static float px=3.,mx=-.3, py=3,my=-3, pz=3,mz=-3;
void magcal() {
  float offset_x, offset_y, offset_z,
        scale_x, scale_y, scale_z,
        total_max, total_min;

  mx = m.mx>mx ? mx + 0.05 : mx;
  my = m.my>my ? my + 0.05 : my;
  mz = m.mz>mz ? mz + 0.05 : mz;
  px = m.mx<px ? px - 0.05 : px;
  py = m.my<py ? py - 0.05 : py;
  pz = m.mz<pz ? pz - 0.05 : pz;

  offset_x = (mx + px) / 2.;
  offset_y = (my + py) / 2.;
  offset_z = (mz + pz) / 2.;

  total_max = max(px, max(py, pz));
  total_min = min(mx, min(my, mz));

  scale_x = (total_max - offset_x) / (mx - offset_x);
  scale_y = (total_max - offset_y) / (my - offset_y);
  scale_z = (total_max - offset_z) / (mz - offset_z);

  m.mx = (m.mx - offset_x) * scale_x;
  m.my = (m.my - offset_y) * scale_y;
  m.mz = (m.mz - offset_z) * scale_z;

  p("s"); p(scale_y); p("\t"); p(offset_y); p("\n");
}
