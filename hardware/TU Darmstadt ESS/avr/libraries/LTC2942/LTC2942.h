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

#ifndef _LTC2942_H_
#define _LTC2942_H_

#include "I2Cdev.h"

#define LTC2942_DEFAULT_ADDRESS   0x64

#define LTC2942_RA_STATUS         0x00
#define LTC2942_RA_CONTROL        0x01
#define LTC2942_RA_CHARGE         0x02
#define LTC2942_RA_VOLTS          0x08
#define LTC2942_RA_TEMP           0x0C

typedef struct {
  uint16_t batteryCharge,
           batteryVoltage,
           temperature;
} ltc2942_measurement_t;

class LTC2942 {
    public:
        LTC2942();

        void initialize();
        bool testConnection();

        // MEASURE1 register, read-only
        void setCharge(uint16_t charge);
        uint16_t getCharge();
        uint16_t getVoltage();
        uint16_t getTemperature();
        ltc2942_measurement_t getMeasurement();

    private:
        uint8_t devAddr;
};

#endif /* _LTC2942_H_ */
