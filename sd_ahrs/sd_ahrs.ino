#include <SPI.h>
#include <SD.h>

#include <I2Cdev.h>
#include <LSM9DS0.h>
#include <ahrs.h>
#include <avr/wdt.h>

/** writes all data to an SDCard, TODO:
**   - load magnetic calibration data from EEPROM
**   - ability to delete the file
**   - realtimestamp?
**   - sleeping while idling
**/

LSM9DS0 sen;
const int chipSelect = 7;

// These are the interrupt and control pins
#define BOOTCHECK(cmd,msg) \
  Serial.print(F(msg"\t..."));\
  if (! (cmd) ) {\
    Serial.println(F("FAIL\n"));\
    goto restart;\
  }\
  Serial.println(F("OK"));

void setup(void)
{ 
restart:
  Serial.begin(115200);
  while(!Serial); // for debugging

  Fastwire::setup(400,true);
  sen.initialize();

  BOOTCHECK( sen.testConnection(), "init LSM9DS0");
  BOOTCHECK( SD.begin(chipSelect), "init SD Card");

  // now configure the sensor
  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_HIGH);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_50);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_100);
}

#define p(x) measurement += String(x);

void loop(void)
{
  static unsigned int lastloop = 0;
  String measurement = "";

  if (micros() - lastloop <= 10*1000) // keep rate at 100Hz == 10ms
    return;

  lastloop = micros();

  lsm9d_measurement_t m  = sen.getMeasurement();
  orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, m.mz,
                  (micros() - lastloop)/1e6);
  o->timestamp = micros();

  if (isnan(o->q0)) { // let's reset
     o->q0 = 1;
     o->q1 = o->q2 = o->q3 = 0;
     o->exInt = o->eyInt = o->ezInt = 0;
     return;
  }

  p(m.ax); p(" ");
  p(m.ay); p(" ");
  p(m.az); p(" ");
  p(m.gx); p(" ");
  p(m.gy); p(" ");
  p(m.gz); p(" ");
  p(m.mx); p(" ");
  p(m.my); p(" ");
  p(m.mz); p(" ");
  p(o->q0); p(" ");
  p(o->q1); p(" ");
  p(o->q2); p(" ");
  p(o->q3); p(" ");
  p(o->timestamp); p("\n");

  File dataFile = SD.open("datalog.txt", FILE_WRITE);

  if (dataFile) {
    dataFile.print(measurement);
    dataFile.close();
  } else {
    Serial.println("error opening file");
  }

  if (Serial && Serial.available() && Serial.read()=='p')
  {
    File dataFile = SD.open("datalog.txt", FILE_READ);

    if (dataFile)
      while(dataFile.available())
        Serial.write(dataFile.read());

    dataFile.close();
  }
}
