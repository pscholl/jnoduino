// stores an calibration array into FLASH EEprom for
// both the magnetometer and accelerometer
//
// calibration is taken from:
//  http://www.bot-thoughts.com/2011/04/quick-and-dirty-compass-calibration-in.html
//  http://www.sensorsmag.com/sensors/motion-velocity-displacement/compensating-tilt-hard-iron-and-soft-iron-effects-6475

#include <I2Cdev.h>
#include <LSM9DS0.h>
#include <EEPROM.h>
#include <math.h>

#define MAGIC 0x33

LSM9DS0 sen;
float inf = INFINITY;

void setup() {
  Serial.begin(115200);
  while(!Serial)
    ; // Wait until there is somebody on the terminal

  Serial.println("move the board in figure eight! press any key to start");
  while(!Serial.available())
    ;
  Serial.read();

  //sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  //sen.setGyroHighPassMode(LSM9DS0_HPM_NORMAL);
  //sen.setGyroHighPassFilterCutOffFrequencyLevel(LSM9DS0_HPCF10);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_362);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_50);

  // clear the EEPROM
  //for (int i = 0; i < 512; i++)
  //  EEPROM.write(i, 0);
}


#define p(x) Serial.print(x)

typedef struct cal {
 float lx, hx, ly, hy, lz, hz;
 float total_max, total_min;
 float offset_x, offset_y, offset_z,
       scale_x, scale_y, scale_z;
} cal_t;

struct cal mag_cal = { 0,0,0,0,0,0, 0,0,0,1,1,1 },
       old_mag_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };
struct cal acc_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 },
       old_acc_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };

uint32_t iteration = 0;

// calculate offset and scale
bool mag_calibrate(float x, float y, float z, struct cal *c)
{
   /* calculate the maximum and minimum like this to avoid having
   ** strong outliers as the min/max values */
   if (x < c->lx) c->lx -= .01;
   if (y < c->ly) c->ly -= .01;
   if (z < c->lz) c->lz -= .01;
   if (x > c->hx) c->hx += .01;
   if (y > c->hy) c->hy += .01;
   if (z > c->hz) c->hz += .01;

   c->offset_x = (c->hx + c->lx) /2.;
   c->offset_y = (c->hy + c->ly) /2.;
   c->offset_z = (c->hz + c->lz) /2.;

   c->total_max = max(c->hx, max(c->hy, c->hz));
   c->total_min = min(c->lx, max(c->ly, c->lz));

   c->scale_x = (c->total_max - c->offset_x) / ( c->hx - c->offset_x);
   c->scale_y = (c->total_max - c->offset_y) / ( c->hy - c->offset_y);
   c->scale_z = (c->total_max - c->offset_z) / ( c->hz - c->offset_z);

   p(c->lx); p(" ");
   p(c->hx); p(" ");
   p(c->ly); p(" ");
   p(c->hy); p(" ");
   p(c->lz); p(" ");
   p(c->hz); p("    ");

   p(c->scale_x); p(" ");
   p(c->scale_y); p(" ");
   p(c->scale_z); p("    ");

   p(c->offset_x); p(" ");
   p(c->offset_y); p(" ");
   p(c->offset_z); p("    ");

   if (old_mag_cal.offset_x == mag_cal.offset_x &&
       old_mag_cal.offset_y == mag_cal.offset_y &&
       old_mag_cal.offset_z == mag_cal.offset_z &&
       old_mag_cal.scale_z == mag_cal.scale_z &&
       old_mag_cal.scale_y == mag_cal.scale_y &&
       old_mag_cal.scale_x == mag_cal.scale_x)
     iteration++;
   else {
     old_mag_cal = mag_cal;
     iteration = 0;
   }

   if (iteration == 1000) {
     uint32_t addr = 0;

     EEPROM.write(addr++,sen.getMagFullScale()+MAGIC);
     for (uint32_t b=0; b<sizeof(struct cal); b++)
       EEPROM.write(addr++, ((char*) &mag_cal)[b]);

     p("\n all done\n");
     return true;
   }

  return false;
}

void loop() {
   static bool mag_calibration_done = false;
   lsm9d_measurement_t m = sen.getMeasurement();

   if (!mag_calibration_done) {
    p("mag "); mag_calibration_done = mag_calibrate(m.mx,m.my,m.mz,&mag_cal); p("\n");
   }
   else { /* do the acc calibration */
    p("all done\n");
    return ;
    p("starting acceleration calibration\n");
    p("x axis high, press any key\n");while(!Serial.available()); Serial.read(); acc_cal.hx = sen.getMeasurement().ax;
    p("x axis low, press any key\n"); while(!Serial.available()); Serial.read(); acc_cal.lx = sen.getMeasurement().ax;
    p("y axis high, press any key\n");while(!Serial.available()); Serial.read(); acc_cal.hy = sen.getMeasurement().ay;
    p("y axis low, press any key\n"); while(!Serial.available()); Serial.read(); acc_cal.ly = sen.getMeasurement().ay;
    p("z axis high, press any key\n");while(!Serial.available()); Serial.read(); acc_cal.hz = sen.getMeasurement().az;
    p("z axis low, press any key\n"); while(!Serial.available()); Serial.read(); acc_cal.lz = sen.getMeasurement().az;

    acc_cal.offset_x = (acc_cal.hx + acc_cal.lx) /2.;
    acc_cal.offset_y = (acc_cal.hy + acc_cal.ly) /2.;
    acc_cal.offset_z = (acc_cal.hz + acc_cal.lz) /2.;
 
    acc_cal.total_max = max(acc_cal.hx, max(acc_cal.hy, acc_cal.hz));
    acc_cal.total_min = min(acc_cal.lx, max(acc_cal.ly, acc_cal.lz));
 
    acc_cal.scale_x = (acc_cal.total_max - acc_cal.offset_x) / ( acc_cal.hx - acc_cal.offset_x);
    acc_cal.scale_y = (acc_cal.total_max - acc_cal.offset_y) / ( acc_cal.hy - acc_cal.offset_y);
    acc_cal.scale_z = (acc_cal.total_max - acc_cal.offset_z) / ( acc_cal.hz - acc_cal.offset_z);
 
    p(acc_cal.lx); p(" ");
    p(acc_cal.hx); p(" ");
    p(acc_cal.ly); p(" ");
    p(acc_cal.hy); p(" ");
    p(acc_cal.lz); p(" ");
    p(acc_cal.hz); p("    ");
 
    p(acc_cal.scale_x); p(" ");
    p(acc_cal.scale_y); p(" ");
    p(acc_cal.scale_z); p("    ");
 
    p(acc_cal.offset_x); p(" ");
    p(acc_cal.offset_y); p(" ");
    p(acc_cal.offset_z); p("    ");

    {
     uint32_t addr = sizeof(struct cal)+1;
     EEPROM.write(addr++,sen.getAccFullScale()+MAGIC);
     for (uint32_t b=0; b<sizeof(struct cal); b++)
       EEPROM.write(addr++, ((char*) &acc_cal)[b]);
    }

p("all done\n");
while(1);
   }
}
