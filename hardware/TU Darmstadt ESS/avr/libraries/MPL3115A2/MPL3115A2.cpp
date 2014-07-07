// I2Cdev library collection - MPL3115A2 I2C device class header file
// Based on Freescale MPL3115A2 datasheet, [02/2013]
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


#include "MPL3115A2.h"

/** Default constructor, uses default I2C address.
 * @see MPL3115A2_DEFAULT_ADDRESS
 */
MPL3115A2::MPL3115A2() {
    devAddr = MPL3115A2_DEFAULT_ADDRESS;
    Fastwire::setup(400,true);
    initialize();
}

/** Power on and prepare for general usage.
 */
void MPL3115A2::initialize() {
  if (!testConnection())
    return;

  // 2Hz + altiemeter mode
  I2Cdev::writeByte(devAddr, MPL3115_RA_CTRL_REG1, 0xB8);
  I2Cdev::writeByte(devAddr, MPL3115_RA_P_DATA_CFG, 0x07);

  // activate
  I2Cdev::writeByte(devAddr, MPL3115_RA_CTRL_REG1, 0xB9);
}

void MPL3115A2::setSampleRate(mpl315_sample_rate_t rate) {
  I2Cdev::writeBits(devAddr, MPL3115_RA_CTRL_REG1, 3, 3, rate);
}

bool MPL3115A2::testConnection() {
  uint8_t tmp;
  return I2Cdev::readByte(devAddr, MPL3115_RA_WHO_AM_I, &tmp) > 0;
}

void MPL3115A2::readSamplesIfNew() {
  uint8_t tmp;
  uint8_t buf[5];

  I2Cdev::readByte(devAddr, 0x00, &tmp);
  if (tmp & 0x08) { // read DATA_READY
    I2Cdev::readBytes(devAddr, 0x01, sizeof(buf), buf);
    alti = ((uint32_t) ((buf[0]<<16)|(buf[1]<<8)|buf[0])) * 0.0625;
    temp = ((uint16_t) ((buf[2]<<8)|buf[3])) * 0.0625;
  }

  alti = tmp;
}

float MPL3115A2::getTemperature() {
  return temp;
}

float MPL3115A2::getAltitude() {
  return alti;
}
