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

#ifndef _MPL3115A2_H_
#define _MPL3115A2_H_

#include "I2Cdev.h"

#define MPL3115A2_DEFAULT_ADDRESS   0xC0

#define MPL3115_RA_WHO_AM_I   0x0C
#define MPL3115_RA_CTRL_REG1  0x26
#define MPL3115_RA_P_DATA_CFG 0x13

typedef enum {
  MPL3115_RATE_166Hz = 0,
  MPL3115_RATE_100Hz,
  MPL3115_RATE_55Hz,
  MPL3115_RATE_30Hz,
  MPL3115_RATE_15Hz,
  MPL3115_RATE_8Hz,
  MPL3115_RATE_4Hz,
  MPL3115_RATE_2Hz
} mpl315_sample_rate_t;

class MPL3115A2 {
    public:
        MPL3115A2();

        void initialize();
        bool testConnection();

        // MEASURE1 register, read-only
        float getTemperature();
        float getAltitude();
        //float getPressure();

        void setSampleRate(mpl315_sample_rate_t rate);

    private:
        void readSamplesIfNew();
        uint8_t devAddr;
        float temp,alti;
};

#endif /* _MPL3115A2_H_ */
