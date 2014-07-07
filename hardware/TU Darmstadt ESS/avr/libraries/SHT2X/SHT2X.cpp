// I2Cdev library collection - SHT2X I2C device class header file
// Based on Sensirion SHT21 datasheet, December 2011
// 13.02.2014 by Philipp M. Scholl <scholl@ess.tu-darmstadt.de>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     [2014-02-13] - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Philipp M. Scholl, Jeff Rowberg

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

#include "SHT2X.h"

SHT2X::SHT2X() {
    devAddr = SHT2X_DEFAULT_ADDRESS;
    Fastwire::setup(400,true);
    initialize();
    last_hum = last_temp = 0;
}

SHT2X::SHT2X(uint8_t address) {
    devAddr = address;
    Fastwire::setup(400,true);
    initialize();
}

bool SHT2X::initialize() {
  bool status = testConnection();

  //if (status)
  //  reset();

  return status;
}

bool SHT2X::testConnection() {
  return true;
}

void SHT2X::reset() {
  //I2Cdev::writeBytes(devAddr, SHT2X_CMD_SOFT_RESET, 0, NULL);
  delay(10); /* wait for reboot */
}

float SHT2X::getTemperature() {
  uint16_t temperature;
  I2Cdev::readWord(devAddr, SHT2X_CMD_NOHOLD_TEMP, &temperature);
  return -46.85 + 175.72 * (temperature / (float) 0xffff);
}

float SHT2X::getRelativeHumidity() {
  uint16_t rh;
  I2Cdev::readWord(devAddr, SHT2X_CMD_NOHOLD_RELH, &rh);
  return -6 + 125 * (rh / (float) 0xffff);
}

void SHT2X::setResolution(uint8_t resolution) {
  uint8_t reg;
  I2Cdev::readByte(devAddr, SHT2X_CMD_READ_USERREG, &reg);
  reg &= ~(SHT2X_RESOLUTION_MASK);
  reg |= resolution;
  I2Cdev::writeByte(devAddr ,SHT2X_CMD_WRITE_USERREG, reg);
}

void SHT2X::setOnChipHeater(bool onoff) {
  uint8_t reg;
  I2Cdev::readByte(devAddr, SHT2X_CMD_READ_USERREG, &reg);
  reg &= ~(1<<SHT2X_ONCHIP_HEATER_BIT);
  reg |= onoff ? (1<<SHT2X_ONCHIP_HEATER_BIT) : 0;
  I2Cdev::writeByte(devAddr ,SHT2X_CMD_WRITE_USERREG, reg);
}
