#include <Adafruit_CC3000.h>
#include <SPI.h>

#include <I2Cdev.h>
#include <LSM9DS0.h>
#include <ahrs.h>

#include <math.h>
#include <avr/io.h>
#include <EEPROM.h>

// These are the interrupt and control pins1g
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

void setup(void)
{ 
restart:
  Serial.begin(115200);
  //while(!Serial); // for debugging

  pinMode(A2, OUTPUT);   // turns green LED on
  digitalWrite(A2, LOW); // wire to GND

  //Serial.println(F("booting UDP motion capture v1.3\n"));

  //BOOTCHECK( sen.testConnection(), "init LSM9DS0");
  //BOOTCHECK( cc3000.begin(),       "init CC3000");
  //BOOTCHECK( cc3000.connectToAP("mocap","",WLAN_SEC_UNSEC), "connecting to mocap" );
  sen.testConnection();
  cc3000.begin();
  cc3000.connectToAP("mocap","",WLAN_SEC_UNSEC);

  //Serial.print(F("requesting DHCP"));
  for (unsigned int i = 0; i < 300 && !cc3000.checkDHCP(); i++)
    delay(100); // 30sec DHCP timeout
  //BOOTCHECK( cc3000.checkDHCP(), "" );
  cc3000.checkDHCP();

  /* Display the IP address DNS, Gateway, etc. */  
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;
  
  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    //Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    goto restart;
  }
  else
  {
//    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
//    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
//    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
//    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
//    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
//    Serial.println();
  }

  // create a broadcast socket
  client = cc3000.connectUDP( ipAddress|~netmask,5050);
  //Serial.println(F("starting to broadcast on UDP port 5050..."));

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
  pinMode(A2, INPUT); // turns green LED off
}

#define RATE_S (1/100.)
#define LIGHTON_DELAY 50
unsigned long lighton=0, time=0, lighton_delay=LIGHTON_DELAY;
lsm9d_measurement_t m;

void loop(void)
{
  if (lighton_delay++ > LIGHTON_DELAY) {
    pinMode(A2, (lighton=!lighton) ? INPUT : OUTPUT);
    lighton_delay = 0;
  }

  m = sen.getMeasurement();
  m.mz = -m.mz; magcal();

  orientation_t *o =
       AHRSupdate(m.gx, m.gy, m.gz,
                  m.ax, m.ay, m.az,
                  m.mx, m.my, m.mz,
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
}
