
/* ATTENTION: need to include Wire and I2CDev libs for
 * jNode sensors (LSM9DS0, MPL115A2, VCNL4010, SHT21X)
 *
 * With this sample we've used 99% of available memory!
 */
#include <Wire.h>
#include <I2Cdev.h>
#include <LSM9DS0.h>

#include <Adafruit_CC3000.h>
#include <SPI.h>
#include "utility/debug.h"
#include "utility/socket.h"

unsigned long time=0;

// These are the interrupt and control pins
// DO NOT CHANGE!!!!  
#define ADAFRUIT_CC3000_IRQ  11
#define ADAFRUIT_CC3000_VBAT 13
#define ADAFRUIT_CC3000_CS   10


Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS,
                                         ADAFRUIT_CC3000_IRQ,
                                         ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER);
                                         
#define WLAN_SSID  "fsr_mobi"  // cannot be longer than 32 characters!
#define WLAN_PASS  "woh0Roo2The7chai"
#define WLAN_SECURITY   WLAN_SEC_WPA2 // Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2

#define LISTEN_PORT 7    // What TCP port to listen on for connections

Adafruit_CC3000_Server quatServer(LISTEN_PORT);
LSM9DS0 sen;
void setup(void)
{
  Wire.begin();
  Serial.begin(115200);

  /* Initialise the module */
  //Serial.println(F("\nInitializing..."));
  if (!cc3000.begin() || (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) )
  {
    Serial.println(F("Failed!"));
    while(1);
  }

  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }

  /*********************************************************/
  /* You can safely remove this to save some flash memory! */
  /*********************************************************/
//  Serial.println(F("\r\nNOTE: This sketch may cause problems with other sketches"));
//  Serial.println(F("since the .disconnect() function is never called, so the"));
//  Serial.println(F("AP may refuse connection requests from the CC3000 until a"));
//  Serial.println(F("timeout period passes.  This is normal behaviour since"));
//  Serial.println(F("there isn't an obvious moment to disconnect with a server.\r\n"));

  // Start listening for connections
  displayConnectionDetails();
  quatServer.begin();

  sen.initialize();

  sen.setGyroFullScale(2000);
  sen.setGyroOutputDataRate(LSM9DS0_RATE_95);
  sen.setGyroBandwidthCutOffMode(LSM9DS0_BW_HIGH);
  sen.setGyroDataFilter(LSM9DS0_LOW_PASS);

  sen.setAccRate(LSM9DS0_ACC_RATE_100);
  sen.setAccFullScale(LSM9DS0_ACC_8G);
  sen.setAccAntiAliasFilterBandwidth(LSM9DS0_ACC_FILTER_BW_50);

  sen.setMagFullScale(LSM9DS0_MAG_4_GAUSS);
  sen.setMagOutputRate(LSM9DS0_M_ODR_100);
}

#define p(x) client.print(x)

void loop(void)
{
  // Try to get a client which is connected.
  Adafruit_CC3000_ClientRef client = quatServer.available();

  if (client) {
     measurement_t m = sen.getMeasurement();

     p(m.ax); p("\t"); // acceleration
     p(m.ay); p("\t");
     p(m.az); p("\t");
     p(m.gx); p("\t"); // gyroscope
     p(m.gy); p("\t");
     p(m.gz); p("\t");
     p(m.mx); p("\t"); // magnetometer
     p(m.my); p("\t");
     p(m.mz); p("\n");
  }
}

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  // There is not enough space to include code for printing the full IP
  // address, so we just print the last byte of it.
  tNetappIpconfigRetArgs ipconfig;
  netapp_ipconfig(&ipconfig);

  Serial.println(ipconfig.aucIP[0]);
}
