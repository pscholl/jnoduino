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

#ifndef _SHT2X_H_
#define _SHT2X_H_

#include "I2Cdev.h"

#define SHT2X_DEFAULT_ADDRESS   0x40

#define SHT2X_CMD_HOLD_TEMP     0xE3
#define SHT2X_CMD_HOLD_RELH     0xE5
#define SHT2X_CMD_NOHOLD_TEMP   0xF3
#define SHT2X_CMD_NOHOLD_RELH   0xF5
#define SHT2X_CMD_WRITE_USERREG 0xE6
#define SHT2X_CMD_READ_USERREG  0xE7
#define SHT2X_CMD_SOFT_RESET    0xFE

#define SHT2X_RESOLUTION_MASK       0x81
#define SHT2X_RESOLUTION_12_14_BITS 0x00
#define SHT2X_RESOLUTION_8_12_BITS  0x01
#define SHT2X_RESOLUTION_10_13_BITS 0x7f
#define SHT2X_RESOLUTION_11_11_BITS 0x81
#define SHT2X_ONCHIP_HEATER_BIT     0x02

class SHT2X {
    public:
        SHT2X();
        SHT2X(uint8_t address);

        bool initialize();
        bool testConnection();
        void reset();

        float getTemperature();
        float getRelativeHumidity();

        void setResolution(uint8_t resolution);
        void setOnChipHeater(bool onoff);

    private:
        uint8_t devAddr;
        float hum,temp;
        uint32_t last_hum,
                 last_temp;
};

#endif /* _SHT2X_H_ */
