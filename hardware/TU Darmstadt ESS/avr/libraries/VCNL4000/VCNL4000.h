// I2Cdev library collection - MYDEVSTUB I2C device class header file
// Based on Vishay VCNL4000 datasheet, [10-May-2012]
// 2013-10-31 by Martin Zittel <martin.zittel@gmail.com>
// Updates should (hopefully) always be available at https://github.com/mzittel/i2cdevlib
//
// Changelog:
//     [2013-10-31] - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 [Author Name], Jeff Rowberg

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

#ifndef _VCNL4000_H_
#define _VCNL4000_H_

#include "I2Cdev.h"


#define VCNL4000_ADDRESS 0x13

#define VCNL4000_RA_COMMAND 0x80
#define VCNL4000_RA_PRODUCTID 0x81
#define VCNL4000_RA_IRLED 0x83
#define VCNL4000_RA_AMBIENTPARAMETER 0x84
#define VCNL4000_RA_AMBIENTDATA 0x85
#define VCNL4000_RA_AMBIENTDATA2 0x86
#define VCNL4000_RA_PROXIMITYDATA 0x87
#define VCNL4000_RA_PROXIMITYDATA2 0x88
#define VCNL4000_RA_SIGNALFREQ 0x89
#define VCNL4000_RA_PROXINITYADJUST 0x8A


#define VCNL4000_MEASUREAMBIENT 0x10    // 0001 0000
#define VCNL4000_MEASUREPROXIMITY 0x08  // 0000 1000
#define VCNL4000_AMBIENTREADY 0x40      // 0100 0000
#define VCNL4000_PROXIMITYREADY 0x20    // 0010 0000


#define VCNL4000_3M125 0
#define VCNL4000_1M5625 1
#define VCNL4000_781K25 2
#define VCNL4000_390K625 3

//typedef enum {
//  VCNL4000_RATE_1_95Hz = 0,
//  VCNL4000_RATE_3_90Hz,
//  VCNL4000_RATE_7_81Hz,
//  VCNL4000_RATE_16_625Hz,
//  VCNL4000_RATE_31_25Hz,
//  VCNL4000_RATE_62_5Hz,
//  VCNL4000_RATE_125Hz,
//  VCNL4000_RATE_250Hz
//} vcnl4000_rate_t;


class VCNL4000 {
    public:
        VCNL4000();
        VCNL4000(uint8_t address);

        void initialize();
        bool testConnection();

        // AmbientLight register, read-only
        uint16_t getAmbientLight();

        // Proximity register, read-only
        uint16_t getProximity();


        void setAmbientLightParameter(uint8_t value);
        uint8_t getAmbientLightParameter();

        void setIrLedCurrent(uint16_t value);
        uint16_t getIrLedCurrent();

        void setProximityMeasurementFrequency(uint8_t rate);
        uint8_t getProximityMeasurementFrequency();

        void setProximityAdjust(uint8_t value);
        uint8_t getProximityAdjust();

        uint8_t getDeviceID();

        void reset();
    private:
        uint8_t devAddr;
        uint8_t buffer[2];
        uint16_t prox,light;
};

#endif /* _VCNL4000_H_ */
