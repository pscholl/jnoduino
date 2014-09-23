#include <Adafruit_CC3000.h>
#include <SPI.h>

#include <I2Cdev.h>
#include <LSM9DS0.h>
#include <ahrs.h>

#include <math.h>
#include <avr/io.h>
#include <EEPROM.h>

// These are the interrupt and control pins
Adafruit_CC3000 cc3000 =  Adafruit_CC3000(10,11,13,SPI_CLOCK_DIVIDER);
Adafruit_CC3000_Client client;
LSM9DS0 sen;

#define BOOTCHECK(cmd,msg) \
  Serial.print(F(msg"\t..."));\
  if (! (cmd) ) {\
    Serial.println(F("FAIL\n"));\
    goto restart;\
  }\
  Serial.println(F("OK"));

/* define calibration data */
typedef struct cal {
 float lx, hx, ly, hy, lz, hz;
 float total_max, total_min;
 float offset_x, offset_y, offset_z,
       scale_x, scale_y, scale_z;
} cal_t;

struct cal mag_cal = { 0,0,0,0,0,0, 0,0,0,1,1,1 };
struct cal acc_cal = { 0,0,0,0,0,0, 0,0,0,1,1,1 };
#define MAGIC 0x33

void setup(void)
{ 
restart:
  Serial.begin(115200);
  //while(!Serial); // for debugging

  pinMode(A2, OUTPUT);   // turns green LED on
  digitalWrite(A2, LOW); // wire to GND

  Serial.println(F("booting UDP motion capture v1.3\n"));

  BOOTCHECK( sen.testConnection(), "init LSM9DS0");
  BOOTCHECK( cc3000.begin(),       "init CC3000");
  BOOTCHECK( cc3000.connectToAP("mocap","",WLAN_SEC_UNSEC), "connecting to mocap" );

  Serial.print(F("requesting DHCP"));
  for (unsigned int i = 0; i < 300 && !cc3000.checkDHCP(); i++)
    delay(100); // 30sec DHCP timeout
  BOOTCHECK( cc3000.checkDHCP(), "" );

  /* Display the IP address DNS, Gateway, etc. */  
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    goto restart;
  }
  else
  {
//    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
//    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
//    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
//    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
//    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
  }

  // create a broadcast socket
  client = cc3000.connectUDP( ipAddress|~netmask,5050);
  Serial.println(F("starting to broadcast on UDP port 5050..."));

  // now configure the sensor
  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_LOW);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_2G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_362);

  sen.setMagFullScale(LSM9DS0_MAG_2_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_50);

  // read calibration data
  if( EEPROM.read(0)-MAGIC == LSM9DS0_MAG_2_GAUSS ) 
  {
    char *p = (char*) &mag_cal;
    for (uint32_t b=0; b<sizeof(struct cal); b++) 
      p[b] = EEPROM.read(b+1);

    p = (char*) &acc_cal;
    for (uint32_t b=0; b<sizeof(struct cal); b++) 
      p[b] = EEPROM.read(sizeof(struct cal)+b+2);
  }


  pinMode(A2, INPUT); // turns green LED off
}

#define RATE_S (1/100.)
#define LIGHTON_DELAY 50
unsigned long lighton=0, time=0, lighton_delay=LIGHTON_DELAY;

void loop(void)
{
  if (lighton_delay++ > LIGHTON_DELAY) {
    pinMode(A2, (lighton=!lighton) ? INPUT : OUTPUT);
    lighton_delay = 0;
  }

  lsm9d_measurement_t m  = sen.getMeasurement();

  m.mx = (m.mx - mag_cal.offset_x) * mag_cal.scale_x;
  m.my = (m.my - mag_cal.offset_y) * mag_cal.scale_y;
  m.mz = (m.mz - mag_cal.offset_z) * mag_cal.scale_z;

  m.ax = (m.ax - acc_cal.offset_x) * acc_cal.scale_x;
  m.ay = (m.ay - acc_cal.offset_y) * acc_cal.scale_y;
  m.az = (m.az - acc_cal.offset_z) * acc_cal.scale_z;

  orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, -m.mz,
                  RATE_S/2); // sampling rate, XXX: make sure to match sensor!
  o->timestamp = micros();

  if (isnan(o->q0)) { // let's reset
     o->q0 = 1;
     o->q1 = o->q2 = o->q3 = 0;
     o->exInt = o->eyInt = o->ezInt = 0;
     if (lighton_delay < LIGHTON_DELAY/2) lighton_delay = LIGHTON_DELAY/2;
     return;
  }

  /* do control the output rate */
  if (micros() - time < RATE_S*1e6)
    return;
  time = micros();

  if ( client.write((uint8_t*) o, sizeof(*o)) < 0 )
    setup(); // emulate reset since wd does not work properly
}
