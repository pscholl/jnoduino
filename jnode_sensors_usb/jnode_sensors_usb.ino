#include <I2Cdev.h>
#include <LTC2942.h>
#include <LSM9DS0.h>
#include <VCNL4000.h>
#include <SHT2X.h>
// #include <MPL3115.h> not working atm

/* This is an example on how to read all sensor and
** send them out in binary over the Serial channel */
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
}

uint16_t cnt;

struct {
  uint32_t            MAGIC;
  uint32_t        timestamp;
  ltc2942_measurement_t ltc;
  lsm9d_measurement_t   lsm;
  float relative_humidity,
        temperature;
  uint16_t ambient_light,
           proximity;
} m = { 0x12345678 };

void loop() {
  m.timestamp = micros();
  m.ltc = ltc.getMeasurement();
  m.lsm = lsm.getMeasurement();

  if ( (cnt++) > 100 ) {
    m.relative_humidity = sht.getRelativeHumidity();
    m.temperature = sht.getTemperature();
    cnt = 0;
  } // since the SHT blocks the bus while sampling, we downsample here

  m.ambient_light = vcnl.getAmbientLight();
  m.proximity = vcnl.getProximity();

  Serial.write((char*) &m);

  if (!Serial)
    setup();
}
