// I2Cdev library collection - VCNL4000 I2C device class header file
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

#include "VCNL4000.h"



/** Default constructor, uses default I2C address.
 * @see VCNL4000_ADDRESS
 */
VCNL4000::VCNL4000() {
    devAddr = VCNL4000_ADDRESS;
    Fastwire::setup(400,true);
    initialize();
}

/** Specific address constructor.
 * @param address I2C address
 * @see VCNL4000_ADDRESS
 */
VCNL4000::VCNL4000(uint8_t address) {
    devAddr = address;
    Fastwire::setup(400,true);
    initialize();
}

/** Power on and prepare for general usage.
 */
void VCNL4000::initialize() {
    setIrLedCurrent(20); // set to 20 * 10mA = 200mA
    setProximityAdjust(0x81); // magic number from Adafruit

    // set automatic measurement, after this no rate changes can be done!
    I2Cdev::writeByte(devAddr,  VCNL4000_RA_COMMAND, 0xe7);
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool VCNL4000::testConnection() {
    if (I2Cdev::readByte(devAddr, VCNL4000_RA_PRODUCTID, buffer) == 1) {
        return true;
    }
    return false;
}

/** Reset the settings.
 * Make sure the device is connected and responds as expected.
 */
void VCNL4000::reset() {
    setIrLedCurrent(20); // set to 20 * 10mA = 200mA
    setProximityAdjust(0x81); // magic number from Adafruit
}


/** Capture the Ambient Light and return the Captured Data.
 * @return Ambient Light Data measured.
 */
uint16_t VCNL4000::getAmbientLight(){
    I2Cdev::readByte( devAddr, VCNL4000_RA_COMMAND, buffer);

    if (buffer[0] & VCNL4000_AMBIENTREADY) {
      I2Cdev::readBytes( devAddr, VCNL4000_RA_AMBIENTDATA, 2, buffer);

      light = buffer[0];
      light <<= 8;
      light = light | buffer[1];
    }

    return light;
}

/** Capture the Proximity and return the Captured Data.
 * @return Ambient Proximity measured.
 */
uint16_t VCNL4000::getProximity(){
    I2Cdev::readByte( devAddr, VCNL4000_RA_COMMAND, buffer);

    if (buffer[0] & VCNL4000_PROXIMITYREADY) {
      I2Cdev::readBytes( devAddr, VCNL4000_RA_PROXIMITYDATA, 2, buffer);

      prox = buffer[0];
      prox <<= 8;
      prox = prox | buffer[1];
    }

    return prox;
}

/** Set ambient light parameters. Averaging and Continous conversion mode.
 * 1. (msb) bit = cont. conversion mode (can be set on for faster measurement)
 * 5. bit = Auto offset compensation (can be set on to compensate package/tempratrue/... related drift)
 * 6.-8. bit = Averaging function (number of measuremants per run)
 * @param ambient light parameters
 */
void VCNL4000::setAmbientLightParameter(uint8_t value){
    I2Cdev::writeByte( devAddr,  VCNL4000_RA_AMBIENTPARAMETER, value);
}

/** Get ambient light parameters. Averaging and Continous conversion mode.
 * 1. (msb) bit = cont. conversion mode (can be set on for faster measurement)
 * 5. bit = Auto offset compensation (can be set on to compensate package/tempratrue/... related drift)
 * 6.-8. bit = Averaging function (number of measuremants per run)
 * @return ambient light parameters
 */
uint8_t VCNL4000::getAmbientLightParameter(){
    I2Cdev::readByte(devAddr, VCNL4000_RA_AMBIENTPARAMETER, buffer);
    return buffer[0];
}

/** Set the current for the IR LED used for the ambient light measurements
 * 1.-2. (msb) bit = Fuse bits (read only)
 * 3.-8. bit = IR-LED current (current = Value * 10mA)
 * @param ir led current
 */
void VCNL4000::setIrLedCurrent(uint16_t value){
    I2Cdev::writeByte( devAddr,  VCNL4000_RA_IRLED, value);
}
/** Set the current for the IR LED used for the ambient light measurements
 * 1.-2. (msb) bit = Fuse bits (read only)
 * 3.-8. bit = IR-LED current (current = Value * 10mA)
 * @return ir led current
 */
uint16_t VCNL4000::getIrLedCurrent(){
    I2Cdev::readByte(devAddr, VCNL4000_RA_IRLED, buffer);
    return buffer[0];
}


/** Set the current for the IR LED used for the ambient light measurements
 * 1.-6. (msb) bit = N/A
 * 7.-8. bit = Proximity frequency (3.125Mhz/1.5625Mhz/781.25Khz/390.625Khz)
 * @see VCNL4000_3M125
 * @see VCNL4000_1M5625
 * @see VCNL4000_781K25
 * @see VCNL4000_390K625
 * @param Proximity frequency
 */
void VCNL4000::setProximityMeasurementFrequency(uint8_t value){
    I2Cdev::writeByte( devAddr,  VCNL4000_RA_SIGNALFREQ, value);
}

/** Set the current for the IR LED used for the ambient light measurements
 * 1.-6. (msb) bit = N/A
 * 7.-8. bit = Proximity frequency (3.125Mhz/1.5625Mhz/781.25Khz/390.625Khz)
 * @see VCNL4000_3M125
 * @see VCNL4000_1M5625
 * @see VCNL4000_781K25
 * @see VCNL4000_390K625
 * @return Proximity frequency
 */
uint8_t VCNL4000::getProximityMeasurementFrequency(){
    I2Cdev::readByte(devAddr, VCNL4000_RA_SIGNALFREQ, buffer);
    return buffer[0];
}

/** Proximity Modulator Timing Adjustment
 * 1.-3. (msb) bit = Modulation delay time (Delay between IR-LED Signal and IR input evaluation)
 * 4.-5. bit = N/A
 * 6.-8. bit = Modulation dead Time (Setting a dead time in evaluation of IR signal at the slopes of the IR signal. 
)
 * @param modulator timing
 */
void VCNL4000::setProximityAdjust(uint8_t value){
    I2Cdev::writeByte( devAddr,  VCNL4000_RA_PROXINITYADJUST, value);
}

/** Proximity Modulator Timing Adjustment
 * 1.-3. (msb) bit = Modulation delay time (Delay between IR-LED Signal and IR input evaluation)
 * 4.-5. bit = N/A
 * 6.-8. bit = Modulation dead Time (Setting a dead time in evaluation of IR signal at the slopes of the IR signal. 
)
 * @return modulator timing
 */
uint8_t VCNL4000::getProximityAdjust(){
    I2Cdev::readByte(devAddr, VCNL4000_RA_PROXINITYADJUST, buffer);
    return buffer[0];
}

// WHO_AM_I register, read-only
uint8_t VCNL4000::getDeviceID(){
    I2Cdev::readByte(devAddr, VCNL4000_RA_PRODUCTID, buffer);
    return buffer[0];
}
