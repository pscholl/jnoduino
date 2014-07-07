#include <I2Cdev.h>
#include <LTC2942.h>
#include <LSM9DS0.h>
#include <VCNL4000.h>
#include <SHT2X.h>
// #include <MPL3115.h> not working atm

/* Example to read all sensors and send out as a
** csv file on the Serial channel. */

LSM9DS0 lsm;
LTC2942 ltc;
VCNL4000 vcnl;
SHT2X sht;

#define p(x) Serial.print(x);

#define BOOTCHECK(cmd,msg) \
  Serial.print(F(msg"\t..."));\
  if (! (cmd) ) {\
    Serial.println(F("FAIL\n"));\
    goto restart;\
  }\
  Serial.println(F("OK"));


void setup() {
restart:
  Serial.begin(115200);

  while (!Serial)
    ;

  BOOTCHECK( ltc.testConnection(), "init LTC2942" );
  BOOTCHECK( lsm.testConnection(), "init LSM9DS0" );
  BOOTCHECK( vcnl.testConnection(), "init VCNL4010" );
  BOOTCHECK( sht.testConnection(), "init SHT2X" );

  p(F("time\taccx\taccy\taccz\tmagx\tmagy\tmagz\tgyrx\tgyry\tgyrz\tlight\tprox\thum\ttemp\tcharge\tvoltage\n"));
}

float rh, temp;
uint16_t cnt;

void loop() {
  ltc2942_measurement_t m = ltc.getMeasurement();
  lsm9d_measurement_t ml = lsm.getMeasurement();

  if ( (cnt++) > 100 ) {
    rh = sht.getRelativeHumidity();
    temp = sht.getTemperature();
    cnt = 0;
  } // since the SHT blocks the bus while sampling, we downsample here

  p(micros()); p(F("\t"));

  p(ml.ax); p(F("\t"));
  p(ml.ay); p(F("\t"));
  p(ml.az); p(F("\t"));
  p(ml.mx); p(F("\t"));
  p(ml.my); p(F("\t"));
  p(ml.mz); p(F("\t"));
  p(ml.gx); p(F("\t"));
  p(ml.gy); p(F("\t"));
  p(ml.gz); p(F("\t"));

  p(vcnl.getAmbientLight()); p(F("\t"));
  p(vcnl.getProximity()); p(F("\t"));

  p(rh); p(F("\t"));
  p(temp); p(F("\t"));

  p(m.batteryCharge); p(F("\t"));
  p(m.batteryVoltage); p(F("\n"));
}
