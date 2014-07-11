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

  //sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  //sen.setGyroHighPassMode(LSM9DS0_HPM_NORMAL);
  //sen.setGyroHighPassFilterCutOffFrequencyLevel(LSM9DS0_HPCF10);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_362);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_50);

  // clear the EEPROM
  for (int i = 0; i < 512; i++)
    EEPROM.write(i, 0);
}


#define p(x) Serial.print(x)

typedef struct cal {
 float lx, hx, ly, hy, lz, hz;
 float total_max, total_min;
 float offset_x, offset_y, offset_z,
       scale_x, scale_y, scale_z;
} cal_t;

struct cal mag_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 },
       old_mag_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };
struct cal acc_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 },
       old_acc_cal = { inf, -inf, inf, -inf, inf, -inf, -inf, inf, 0,0,0,1,1,1 };

uint32_t iteration = 0;

// calculate offset and scale
void calibrate(float x, float y, float z, struct cal *c)
{
   c->lx = min(c->lx, x);
   c->ly = min(c->ly, y);
   c->lz = min(c->lz, z);
   c->hx = max(c->hx, x);
   c->hy = max(c->hy, y);
   c->hz = max(c->hz, z);

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
     while(1)
       ;
   }
}

void loop() {
   lsm9d_measurement_t m = sen.getMeasurement();

   // for now only magnetometer
   p("mag "); calibrate(m.mx,m.my,m.mz,&mag_cal); p("\n");
}


