// I2Cdev library collection - LTC2942 I2C device class header file
// Based on Freescale LTC2942 datasheet, [02/2013]
// 2014-02-11 by Philipp M. Scholl <scholl@ess.tu-darmstadt.de>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     [2014-02-11] - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011-2014 Philipp M. Scholl, Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/


#include "LTC2942.h"

/** Default constructor, uses default I2C address.
 * @see LTC2942_DEFAULT_ADDRESS
 */
LTC2942::LTC2942() {
    devAddr = LTC2942_DEFAULT_ADDRESS;
    Fastwire::setup(400,true);
    initialize();
}

/** Power on and prepare for general usage.
 */
void LTC2942::initialize() {
  if (!testConnection())
    return;

  /* power up to sample every 2s */
  I2Cdev::writeByte(devAddr, LTC2942_RA_CONTROL, 0xF8);
}

bool LTC2942::testConnection() {
  uint8_t tmp;
  return I2Cdev::readByte(devAddr, LTC2942_RA_STATUS, &tmp) > 0;
}

void LTC2942::setCharge(uint16_t charge) {
  // TODO: shutdown analog conversion!
  I2Cdev::writeWord(devAddr, LTC2942_RA_CHARGE, charge);
}

uint16_t LTC2942::getCharge() {
  uint8_t buf[LTC2942_RA_CHARGE+2]; // this chip ignores the register address!
  I2Cdev::readBytes(devAddr, 0, sizeof(buf), buf);
  return (buf[LTC2942_RA_CHARGE]<<8)|buf[LTC2942_RA_CHARGE+1];
}

uint16_t LTC2942::getVoltage() {
  uint8_t buf[LTC2942_RA_VOLTS+2]; // this chip ignores the register address!
  I2Cdev::readBytes(devAddr, 0, sizeof(buf), buf);
  return (buf[LTC2942_RA_VOLTS]<<8)|buf[LTC2942_RA_VOLTS+1];
}

uint16_t LTC2942::getTemperature() {
  uint8_t buf[LTC2942_RA_TEMP+2]; // this chip ignores the register address!
  I2Cdev::readBytes(devAddr, 0, sizeof(buf), buf);
  return (buf[LTC2942_RA_TEMP]<<8)|buf[LTC2942_RA_TEMP+1];
}

ltc2942_measurement_t LTC2942::getMeasurement() {
  uint8_t buf[16];
  ltc2942_measurement_t m;
  I2Cdev::readBytes(devAddr, 0, sizeof(buf), buf);
  m.batteryCharge = (buf[LTC2942_RA_CHARGE]<<8)|buf[LTC2942_RA_CHARGE+1];
  m.batteryVoltage = (buf[LTC2942_RA_VOLTS]<<8)|buf[LTC2942_RA_VOLTS+1];
  m.temperature = (buf[LTC2942_RA_TEMP]<<8)|buf[LTC2942_RA_TEMP+1];
  return m;
}
