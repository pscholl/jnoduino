#include <Adafruit_CC3000.h>
#include <SPI.h>

#include <I2Cdev.h>
#include <LTC2942.h>
#include <LSM9DS0.h>
#include <VCNL4000.h>
#include <SHT2X.h>
// #include <MPL3115.h> not working atm

/* This example connects to the configured WiFi network and
** sends out all sensor on UDP Broadcast channel */

// These are the interrupt and control pins
Adafruit_CC3000 cc3000 =  Adafruit_CC3000(10,11,13,SPI_CLOCK_DIVIDER);
Adafruit_CC3000_Client client;
LSM9DS0 lsm;
LTC2942 ltc;
VCNL4000 vcnl;
SHT2X sht;


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

  Serial.println(F("booting UDP motion capture v1.2\n"));

  BOOTCHECK( ltc.testConnection(), "init LTC2942" );
  BOOTCHECK( lsm.testConnection(), "init LSM9DS0" );
  BOOTCHECK( vcnl.testConnection(), "init VCNL4010" );
  BOOTCHECK( sht.testConnection(), "init SHT2X" );
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
}

uint32_t lastloop=0;
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


void loop(void)
{
  if (micros() - lastloop <= 10*1000) // keep rate at 100Hz == 10ms
    return;

  lastloop = micros();

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

  if ( client.write((uint8_t*) &m, sizeof(m)) < 0 )
    setup(); // emulate reset since wd does not work properly
}
