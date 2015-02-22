// I2Cdev library collection - LSM9DS0 I2C device class
// Based on STMicroelectronics LSM9DS0 datasheet rev. 3, 12/2010
// 7/31/2013 by Jonathan Arnett <j3rn@j3rn.com>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//     2013-07-31 - initial release

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2011 Jonathan Arnett, Jeff Rowberg

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

#include "LSM9DS0.h"
#include <math.h>

/** Default constructor, uses default I2C address.
 * @see LSM9DS0_DEFAULT_ADDRESS
 */
LSM9DS0::LSM9DS0() {
    devAddrGyro = LSM9DS0_GYRO_DEFAULT_ADDRESS;
    devAddrMagAcc = LSM9DS0_ACCMAG_DEFAULT_ADDRESS;
    Fastwire::setup(400,true);
    initialize();
}

/** Specific address constructor.
 * @param address I2C address
 * @see LSM9DS0_DEFAULT_ADDRESS
 * @see LSM9DS0_ADDRESS
 */
LSM9DS0::LSM9DS0(uint8_t address, uint8_t accmag) {
    devAddrGyro = address;
    devAddrMagAcc = accmag;
    Fastwire::setup(400,true);
    initialize();
}

/** Power on and prepare for general usage.
 * All values are defaults except for the power on bit in CTRL_REG_1
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_RA_CTRL_REG2
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_RA_CTRL_REG5
 */
void LSM9DS0::initialize() {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_CTRL_REG1, 0b00001111);
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_CTRL_REG2, 0b00000000);
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_CTRL_REG3, 0b00000000);
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_CTRL_REG4, 0b10000000);
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_CTRL_REG5, 0b00000000);

    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, 0b00000000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG1_XM, 0b01101111);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG2_XM, 0b00000000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG3_XM, 0b00000000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG4_XM, 0b10000000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG5_XM, 0b10010000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG6_XM, 0b00100000);
    I2Cdev::writeByte(devAddrMagAcc, LSM9DS0_RA_CTRL_REG7_XM, 0b00000000);

    acc_scale = 0.061 / 1000. * 9.81;
    mag_scale = 0.16  / 1000.;
    gyr_scale = 8.75  / 1000.;
}

/** Verify the I2C connection.
 * Make sure the device is connected and responds as expected.
 * @return True if connection is valid, false otherwise
 */
bool LSM9DS0::testConnection() {
    return (getDeviceID() == 0b11010100) &&
           (getDeviceIDMagAcc() == 0b01001001);
}

// WHO_AM_I register, read-only

/** Get the Device ID.
 * The WHO_AM_I register holds the device's id
 * @return Device ID (should be 0b11010011, 109, 0x69)
 * @see LSM9DS0_RA_WHO_AM_I
 */
uint8_t LSM9DS0::getDeviceID() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_WHO_AM_I, buffer);
    return buffer[0];
}

uint8_t LSM9DS0::getDeviceIDMagAcc() {
    I2Cdev::readByte(devAddrMagAcc, LSM9DS0_RA_WHO_AM_I, buffer);
    return buffer[0];
}

// CTRL_REG1 register, r/w

/** Set the output data rate
 * @param rate The new data output rate (can be 100, 200, 400, or 800)
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_ODR_BIT
 * @see LSM9DS0_ODR_LENGTH
 * @see LSM9DS0_RATE_100
 * @see LSM9DS0_RATE_200
 * @see LSM9DS0_RATE_400
 * @see LSM9DS0_RATE_800
 */
void LSM9DS0::setGyroOutputDataRate(gyro_rate_t rate) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_ODR_BIT,
        LSM9DS0_ODR_LENGTH, rate);
}

/** Get the current output data rate
 * @return Current data output rate
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_ODR_BIT
 * @see LSM9DS0_ODR_LENGTH
 * @see LSM9DS0_RATE_100
 * @see LSM9DS0_RATE_200
 * @see LSM9DS0_RATE_400
 * @see LSM9DS0_RATE_800
 */
gyro_rate_t LSM9DS0::getGyroOutputDataRate() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_ODR_BIT, 
        LSM9DS0_ODR_LENGTH, buffer);
    uint8_t rate = buffer[0];
    return (gyro_rate_t) rate;
}

/** Set the bandwidth cut-off mode
 * @param mode The new bandwidth cut-off mode
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_BW_BIT
 * @see LSM9DS0_BW_LENGTH
 * @see LSM9DS0_BW_LOW
 * @see LSM9DS0_BW_MED_LOW
 * @see LSM9DS0_BW_MED_HIGH
 * @see LSM9DS0_BW_HIGH
 */
void LSM9DS0::setGyroBandwidthCutOffMode(gyro_bandwidth_t mode) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_BW_BIT,
        LSM9DS0_BW_LENGTH, mode);
}

/** Get the current bandwidth cut-off mode
 * @return Current bandwidth cut off mode
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_BW_BIT
 * @see LSM9DS0_BW_LENGTH
 * @see LSM9DS0_BW_LOW
 * @see LSM9DS0_BW_MED_LOW
 * @see LSM9DS0_BW_MED_HIGH
 * @see LSM9DS0_BW_HIGH
 */
gyro_bandwidth_t LSM9DS0::getGyroBandwidthCutOffMode() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_BW_BIT, 
        LSM9DS0_BW_LENGTH, buffer);
    return (gyro_bandwidth_t) buffer[0];
}

/** Gets the current bandwidth cutoff based on ODR and BW
 * @return Float value of the bandwidth cut off
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_ODR_BIT
 * @see LSM9DS0_ODR_LENGTH
 * @see LSM9DS0_RATE_100
 * @see LSM9DS0_RATE_200
 * @see LSM9DS0_RATE_400
 * @see LSM9DS0_RATE_800
 * @see LSM9DS0_BW_BIT
 * @see LSM9DS0_BW_LENGTH
 * @see LSM9DS0_BW_LOW
 * @see LSM9DS0_BW_MED_LOW
 * @see LSM9DS0_BW_MED_HIGH
 * @see LSM9DS0_BW_HIGH
 */
float LSM9DS0::getGyroBandwidthCutOff() {
    uint16_t dataRate = getGyroOutputDataRate();
    uint8_t bandwidthMode = getGyroBandwidthCutOffMode();

    if (dataRate == 100) {
        if (bandwidthMode == LSM9DS0_BW_LOW) {
            return 12.5;
        } else {
            return 25.0;
        }
    } else if (dataRate == 200) {
        if (bandwidthMode == LSM9DS0_BW_LOW) {
            return 12.5;
        } else if (bandwidthMode == LSM9DS0_BW_MED_LOW) {
            return 25.0;
        } else if (bandwidthMode == LSM9DS0_BW_MED_HIGH) {
            return 50.0;
        } else {
            return 70.0;
        }
    } else if (dataRate == 400) {
        if (bandwidthMode == LSM9DS0_BW_LOW) {
            return 20.0;
        } else if (bandwidthMode == LSM9DS0_BW_MED_LOW) {
            return 25.0;
        } else if (bandwidthMode == LSM9DS0_BW_MED_HIGH) {
            return 50.0;
        } else {
            return 110.0;
        }
    } else {
        if (bandwidthMode == LSM9DS0_BW_LOW) {
            return 30.0;
        } else if (bandwidthMode == LSM9DS0_BW_MED_LOW) {
            return 35.0;
        } else if (bandwidthMode == LSM9DS0_BW_MED_HIGH) {
            return 50.0;
        } else {
            return 110.0;
        }
    }
}

/** Set power on or off
 * @param enabled The new power setting (true for on, false for off)
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_PD_BIT
 */
void LSM9DS0::setGyroPowerOn(bool on) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_PD_BIT, on);
}

/** Get the current power state
 * @return Powered on state (true for on, false for off)
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_PD_BIT
 */
bool LSM9DS0::getGyroPowerOn() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_PD_BIT, buffer);
    return buffer[0];
}

/** Enables or disables the ability to get Z data
 * @param enabled The new enabled state of the Z axis
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_ZEN_BIT
 */
void LSM9DS0::setGyroZEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_ZEN_BIT, enabled);
}

/** Get whether Z axis data is enabled
 * @return True if the Z axis is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_ZEN_BIT
 */
bool LSM9DS0::getGyroZEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_ZEN_BIT, buffer);
    return buffer[0];
}

/** Enables or disables the ability to get Y data
 * @param enabled The new enabled state of the Y axis
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_YEN_BIT
 */
void LSM9DS0::setGyroYEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_YEN_BIT, enabled);
}

/** Get whether Y axis data is enabled
 * @return True if the Y axis is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_YEN_BIT
 */
bool LSM9DS0::getGyroYEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_YEN_BIT, buffer);
    return buffer[0];
}

/** Enables or disables the ability to get X data
 * @param enabled The new enabled state of the X axis
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_XEN_BIT
 */
void LSM9DS0::setGyroXEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_XEN_BIT, enabled);
}

/** Get whether X axis data is enabled
 * @return True if the X axis is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG1
 * @see LSM9DS0_XEN_BIT
 */
bool LSM9DS0::getGyroXEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG1, LSM9DS0_XEN_BIT, buffer);
    return buffer[0];
}

// CTRL_REG2 register, r/w

/** Set the high pass mode
 * @param mode The new high pass mode
 * @see LSM9DS0_RA_CTRL_REG2
 * @see LSM9DS0_HPM_BIT
 * @see LSM9DS0_HPM_LENGTH
 * @see LSM9DS0_HPM_HRF
 * @see LSM9DS0_HPM_REFERENCE
 * @see LSM9DS0_HPM_NORMAL
 * @see LSM9DS0_HPM_AUTORESET
 */
void LSM9DS0::setGyroHighPassMode(uint8_t mode) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG2, LSM9DS0_HPM_BIT, 
        LSM9DS0_HPM_LENGTH, mode);
}

/** Get the high pass mode
 * @return High pass mode
 * @see LSM9DS0_RA_CTRL_REG2
 * @see LSM9DS0_HPM_BIT
 * @see LSM9DS0_HPM_LENGTH
 * @see LSM9DS0_HPM_HRF
 * @see LSM9DS0_HPM_REFERENCE
 * @see LSM9DS0_HPM_NORMAL
 * @see LSM9DS0_HPM_AUTORESET
 */
uint8_t LSM9DS0::getGyroHighPassMode() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG2, LSM9DS0_HPM_BIT, 
        LSM9DS0_HPM_LENGTH, buffer);
    return buffer[0];
}

/** Set the high pass filter cut off frequency level (1 - 10)
 * @param level The new level for the hpcf, using one of the defined levels
 * @see LSM9DS0_RA_CTRL_REG2
 * @see LSM9DS0_HPCF_BIT
 * @see LSM9DS0_HPCF_LENGTH
 * @see LSM9DS0_HPCF1
 * @see LSM9DS0_HPCF2
 * @see LSM9DS0_HPCF3
 * @see LSM9DS0_HPCF4
 * @see LSM9DS0_HPCF5
 * @see LSM9DS0_HPCF6
 * @see LSM9DS0_HPCF7
 * @see LSM9DS0_HPCF8
 * @see LSM9DS0_HPCF9
 * @see LSM9DS0_HPCF10
 */
void LSM9DS0::setGyroHighPassFilterCutOffFrequencyLevel(uint8_t level) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG2, LSM9DS0_HPCF_BIT, 
        LSM9DS0_HPCF_LENGTH, level);
}

/** Get the high pass filter cut off frequency level (1 - 10)
 * @return High pass filter cut off frequency level
 * @see LSM9DS0_RA_CTRL_REG2
 * @see LSM9DS0_HPCF_BIT
 * @see LSM9DS0_HPCF_LENGTH
 * @see LSM9DS0_HPCF1
 * @see LSM9DS0_HPCF2
 * @see LSM9DS0_HPCF3
 * @see LSM9DS0_HPCF4
 * @see LSM9DS0_HPCF5
 * @see LSM9DS0_HPCF6
 * @see LSM9DS0_HPCF7
 * @see LSM9DS0_HPCF8
 * @see LSM9DS0_HPCF9
 * @see LSM9DS0_HPCF10
 */
uint8_t LSM9DS0::getGyroHighPassFilterCutOffFrequencyLevel() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG2, LSM9DS0_HPCF_BIT, 
        LSM9DS0_HPCF_LENGTH, buffer);
    return buffer[0];
}

// CTRL_REG3 register, r/w

/** Set the INT1 interrupt enabled state
 * @param enabled New enabled state for the INT1 interrupt
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I1_INT1_BIT
 */
void LSM9DS0::setGyroINT1InterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I1_INT1_BIT, 
        enabled);
}

/** Get the INT1 interrupt enabled state
 * @return True if the INT1 interrupt is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I1_INT1_BIT
 */
bool LSM9DS0::getGyroINT1InterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I1_INT1_BIT, 
        buffer);
    return buffer[0];
}

/** Set the INT1 boot status enabled state
 * @param enabled New enabled state for the INT1 boot status
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I1_BOOT_BIT
 */
void LSM9DS0::setGyroINT1BootStatusEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I1_BOOT_BIT, 
        enabled);
}

/** Get the INT1 boot status enabled state
 * @return INT1 boot status status
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I1_BOOT_BIT
 */
bool LSM9DS0::getGyroINT1BootStatusEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I1_BOOT_BIT, 
        buffer);
    return buffer[0];
}

/** Interrupts the active INT1 configuration
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_H_LACTIVE_BIT
 */
void LSM9DS0::interruptActiveINT1Config() {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_H_LACTIVE_BIT, 1);
}

/** Set output mode to push-pull or open-drain
 * @param mode New output mode
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_PP_OD_BIT
 * @see LSM9DS0_PUSH_PULL
 * @see LSM9DS0_OPEN_DRAIN
 */
void LSM9DS0::setGyroOutputMode(bool mode) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_PP_OD_BIT, 
        mode);
}

/** Get whether mode is push-pull or open drain
 * @return Output mode (TRUE for push-pull, FALSE for open-drain)
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_PP_OD_BIT
 * @see LSM9DS0_PUSH_PULL
 * @see LSM9DS0_OPEN_DRAIN
 */
bool LSM9DS0::getGyroOutputMode() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_PP_OD_BIT, 
        buffer);
    return buffer[0];
}

/** Set data ready interrupt enabled state on INT2 pin
 * @param enabled New INT2 data ready interrupt enabled state
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_DRDY_BIT
 */
void LSM9DS0::setGyroINT2DataReadyEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_DRDY_BIT, 
        enabled);
}

/** Get whether the data ready interrupt is enabled on the INT2 pin
 * @return True if the INT2 data ready interrupt is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_DRDY_BIT
 */
bool LSM9DS0::getGyroINT2DataReadyEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_DRDY_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the INT2 FIFO watermark interrupt is enabled
 * The sensor contains a 32-slot FIFO buffer for storing data so that it may be 
 * read later. If enabled, the sensor will generate an interrupt on the 
 * INT2/DRDY pin when the watermark has been reached. The watermark can be 
 * configured through the setFIFOWatermark function.
 * @param enabled New enabled state of the INT2 FIFO watermark
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_WTM_BIT
 */
void LSM9DS0::setGyroINT2FIFOWatermarkInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_WTM_BIT, 
        enabled);
}

/** Get the INT2 FIFO watermark interrupt enabled state
 * @return true if the FIFO watermark is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_WTM_BIT
 */ 
bool LSM9DS0::getGyroINT2FIFOWatermarkInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_WTM_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether an interrupt is triggered on INT2 when the FIFO is overrun
 * @param enabled New FIFO overrun interrupt enabled state
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_ORUN_BIT
 */
void LSM9DS0::setGyroINT2FIFOOverrunInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_ORUN_BIT, 
        enabled);
}

/** Get whether an interrupt is triggered on INT2 when the FIFO is overrun
 * @return True if the INT2 FIFO overrun interrupt is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_ORUN_BIT
 */
bool LSM9DS0::getGyroINT2FIFOOverrunInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_ORUN_BIT,
        buffer);
    return buffer[0];
}

/** Set whether an interrupt is triggered on INT2 when the FIFO buffer is empty
 * @param enabled New INT2 FIFO empty interrupt state
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_EMPTY_BIT
 */
void LSM9DS0::setGyroINT2FIFOEmptyInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_EMPTY_BIT, 
        enabled);
}

/** Get whether the INT2 FIFO empty interrupt is enabled
 * @returns Trur if the INT2 FIFO empty interrupt is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG3
 * @see LSM9DS0_I2_EMPTY_BIT
 */
bool LSM9DS0::getGyroINT2FIFOEmptyInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG3, LSM9DS0_I2_EMPTY_BIT, 
        buffer);
    return buffer[0];
}

// CTRL_REG4 register, r/w

/** Set the Block Data Update (BDU) enabled state
 * @param enabled New BDU enabled state
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_BDU_BIT
 */
void LSM9DS0::setGyroBlockDataUpdateEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_BDU_BIT, enabled);
}

/** Get the BDU enabled state
 * @return True if Block Data Update is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_BDU_BIT
 */
bool LSM9DS0::getGyroBlockDataUpdateEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_BDU_BIT, buffer);
    return buffer[0];
}

/** Set the data endian modes
 * In Big Endian mode, the Most Significat Byte (MSB) is on the lower address, 
 * and the Least Significant Byte (LSB) is on the higher address. Little Endian 
 * mode reverses this order. Little Endian is the default mode.
 * @param endianness New endian mode
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_BLE_BIT
 * @see LSM9DS0_BIG_ENDIAN
 * @see LSM9DS0_LITTLE_ENDIAN
 */
void LSM9DS0::setGyroEndianMode(bool endianness) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_BLE_BIT, 
        endianness);
}

/** Get the data endian mode
 * @return Current endian mode
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_BLE_BIT
 * @see LSM9DS0_BIG_ENDIAN
 * @see LSM9DS0_LITTLE_ENDIAN
 */
bool LSM9DS0::getGyroEndianMode() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_BLE_BIT,
        buffer);
    return buffer[0];
}

/** Set the full scale of the data output (in dps)
 * @param scale The new scale of the data output (250, 500, 2000)
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_FS_BIT
 * @see LSM9DS0_FS_LENGTH
 * @see LSM9DS0_FS_250
 * @see LSM9DS0_FS_500
 * @see LSM9DS0_FS_2000
 */
void LSM9DS0::setGyroFullScale(uint16_t scale) {
    uint8_t writeBits;

    if (scale == 250) {
        writeBits = LSM9DS0_FS_250;
        gyr_scale = 8.75 / 1000.;
    } else if (scale == 500) {
        writeBits = LSM9DS0_FS_500;
        gyr_scale = 17.5 / 1000.;
    } else {
        writeBits = LSM9DS0_FS_2000;
        gyr_scale = 70 / 1000.;
    }

    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_FS_BIT, 
        LSM9DS0_FS_LENGTH, writeBits);
}

/** Get the current full scale of the output data (in dps)
 * @return Current scale of the output data
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_FS_BIT
 * @see LSM9DS0_FS_LENGTH
 * @see LSM9DS0_FS_250
 * @see LSM9DS0_FS_500
 * @see LSM9DS0_FS_2000
 */
uint16_t LSM9DS0::getGyroFullScale() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG4, 
        LSM9DS0_FS_BIT, LSM9DS0_FS_LENGTH, buffer);
    uint8_t readBits = buffer[0];

    if (readBits == LSM9DS0_FS_250) {
        return 250;
    } else if (readBits == LSM9DS0_FS_500) {
        return 500;
    } else {
        return 2000;
    }
}

/** Set the self test mode
 * @param mode New self test mode (Normal, 0, 1)
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_ST_BIT
 * @see LSM9DS0_ST_LENGTH
 * @see LSM9DS0_SELF_TEST_NORMAL
 * @see LSM9DS0_SELF_TEST_0
 * @see LSM9DS0_SELF_TEST_1
 */
void LSM9DS0::setGyroSelfTestMode(uint8_t mode) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_ST_BIT, 
        LSM9DS0_ST_LENGTH, mode);
}

/** Get the current self test mode
 * @return Current self test mode
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_ST_BIT
 * @see LSM9DS0_ST_LENGTH
 * @see LSM9DS0_SELF_TEST_NORMAL
 * @see LSM9DS0_SELF_TEST_0
 * @see LSM9DS0_SELF_TEST_1
 */
uint8_t LSM9DS0::getGyroSelfTestMode() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_ST_BIT, 
        LSM9DS0_ST_LENGTH, buffer);
    return buffer[0];
}

/** Set the SPI mode
 * @param mode New SPI mode
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_SIM_BIT
 * @see LSM9DS0_SPI_4_WIRE
 * @see LSM9DS0_SPI_3_WIRE
 */
void LSM9DS0::setGyroSPIMode(bool mode) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_SIM_BIT, mode);
}

/** Get the SPI mode
 * @return Current SPI mode
 * @see LSM9DS0_RA_CTRL_REG4
 * @see LSM9DS0_SIM_BIT
 * @see LSM9DS0_SPI_4_WIRE
 * @see LSM9DS0_SPI_3_WIRE
 */
bool LSM9DS0::getGyroSPIMode() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG4, LSM9DS0_SIM_BIT, 
        buffer);
    return buffer[0];
}

// CTRL_REG5 register, r/w

/** Reboots the FIFO memory content
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_BOOT_BIT
 */
void LSM9DS0::rebootMemoryContent() {
  I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_BOOT_BIT, true);
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_BOOT_BIT, true);
}

/** Set whether the FIFO buffer is enabled
 * @param enabled New enabled state of the FIFO buffer
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_FIFO_EN_BIT
 */
void LSM9DS0::setFIFOEnabled(bool enabled) {
  I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_FIFO_EN_BIT, enabled);
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_FIFO_EN_BIT, enabled);
}

/** Get whether the FIFO buffer is enabled
 * @return True if the FIFO buffer is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_FIFO_EN_BIT
 */
bool LSM9DS0::getFIFOEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_FIFO_EN_BIT, buffer);
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_FIFO_EN_BIT, &buffer[1]);
    return buffer[0] && buffer[1];
}

/** Set the high pass filter enabled state
 * @param enabled New high pass filter enabled state
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_HPEN_BIT
 */
void LSM9DS0::setGyroHighPassFilterEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_HPEN_BIT, 
        enabled);
}

/** Get whether the high pass filter is enabled
 * @return True if the high pass filter is enabled, false otherwise
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_HPEN_BIT
 */
bool LSM9DS0::getGyroHighPassFilterEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_HPEN_BIT,
        buffer);
    return buffer[0];
}

/** Sets the filter mode to one of the four provided.
 * This function also uses the setHighPassFilterEnabled function in order to set
 * the mode. That function does not haved to be called in addition to this one. 
 * In addition to setting the filter for the data in the FIFO buffer 
 * (controlled by the bits written to OUT_SEL), this function also sets the
 * filter used for interrupt generation (the bits written to INT1_SEL) to be the
 * same as the filter used for the FIFO buffer.
 * @param filter New method to be used when filtering data
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_INT1_SEL_BIT
 * @see LSM9DS0_INT1_SEL_LENGTH
 * @see LSM9DS0_OUT_SEL_BIT
 * @see LSM9DS0_OUT_SEL_LENGTH
 * @see LSM9DS0_NON_HIGH_PASS
 * @see LSM9DS0_HIGH_PASS
 * @see LSM9DS0_LOW_PASS
 * @see LSM9DS0_LOW_HIGH_PASS
 */
void LSM9DS0::setGyroDataFilter(uint8_t filter) {
    if (filter == LSM9DS0_HIGH_PASS || filter == LSM9DS0_LOW_HIGH_PASS) {
        setGyroHighPassFilterEnabled(true);
    } else {
        setGyroHighPassFilterEnabled(false);
    }

    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_OUT_SEL_BIT, 
        LSM9DS0_OUT_SEL_LENGTH, filter);
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_INT1_SEL_BIT, 
        LSM9DS0_INT1_SEL_LENGTH, filter);
}

/** Gets the data filter currently in use
 * @return Defined value that represents the filter in use
 * @see LSM9DS0_RA_CTRL_REG5
 * @see LSM9DS0_OUT_SEL_BIT
 * @see LSM9DS0_OUT_SEL_LENGTH
 * @see LSM9DS0_NON_HIGH_PASS
 * @see LSM9DS0_HIGH_PASS
 * @see LSM9DS0_LOW_PASS
 * @see LSM9DS0_LOW_HIGH_PASS
 */
uint8_t LSM9DS0::getGyroDataFilter() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_CTRL_REG5, LSM9DS0_OUT_SEL_BIT, 
        LSM9DS0_OUT_SEL_LENGTH, buffer);
    uint8_t outBits = buffer[0];

    if (outBits == LSM9DS0_NON_HIGH_PASS || outBits == LSM9DS0_HIGH_PASS) {
        return outBits;
    }

    if (getGyroHighPassFilterEnabled()) {
        return LSM9DS0_LOW_HIGH_PASS;
    } else {
        return LSM9DS0_LOW_PASS;
    }
}

// REFERENCE/DATACAPTURE register, r/w

/** Set the reference value for interrupt generation
 * @param reference New reference value for interrupt generation
 * @see LSM9DS0_RA_REFERENCE
 */
void LSM9DS0::setGyroInterruptReference(uint8_t reference) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_REFERENCE, reference);
}

/** Get the 8-bit reference value for interrupt generation
 * @return 8-bit reference value for interrupt generation
 * @see LSM9DS0_RA_REFERENCE
 */
uint8_t LSM9DS0::getGyroInterruptReference() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_REFERENCE, buffer);
    return buffer[0];
}

// STATUS register, read-only

/** Get whether new data overwrote the last set of data before it was read
 * @return True if the last set of data was overwritten before being read, false
 * otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_ZYXOR_BIT
 */
bool LSM9DS0::getGyroXYZOverrun() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_ZYXOR_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether new Z data overwrote the last set of data before it was read
 * @return True if the last set of Z data was overwritten before being read,
 * false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_ZOR_BIT
 */
bool LSM9DS0::getGyroZOverrun() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_ZOR_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether new Y data overwrote the last set of data before it was read
 * @return True if the last set of Y data was overwritten before being read, 
 * false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_YOR_BIT
 */
bool LSM9DS0::getGyroYOverrun() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_YOR_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether new X data overwrote the last set of data before it was read
 * @return True if the last set of X data was overwritten before being read, 
 * false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_XOR_BIT
 */
bool LSM9DS0::getGyroXOverrun() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_XOR_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether there is new data avaialable
 * @return True if there is new data available, false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_ZYXDA_BIT
 */
bool LSM9DS0::getGyroXYZDataAvailable() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_ZYXDA_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether there is new Z data avaialable
 * @return True if there is new Z data available, false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_ZDA_BIT
 */
bool LSM9DS0::getGyroZDataAvailable() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_ZDA_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether there is new Y data avaialable
 * @return True if there is new Y data available, false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_YDA_BIT
 */
bool LSM9DS0::getGyroYDataAvailable() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_YDA_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether there is new X data avaialable
 * @return True if there is new X data available, false otherwise
 * @see LSM9DS0_RA_STATUS
 * @see LSM9DS0_XDA_BIT
 */
bool LSM9DS0::getGyroXDataAvailable() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_STATUS, LSM9DS0_XDA_BIT, 
        buffer);
    return buffer[0];
}

// OUT_* registers, read-only

/** Get the angular velocity for all 3 axes
 * Due to the fact that this device supports two difference Endian modes, both 
 * must be accounted for when reading data. In Little Endian mode, the first 
 * byte (lowest address) is the least significant and in Big Endian mode the 
 * first byte is the most significant.
 * @param x 16-bit integer container for the X-axis angular velocity
 * @param y 16-bit integer container for the Y-axis angular velocity
 * @param z 16-bit integer container for the Z-axis angular velocity
 */
void LSM9DS0::getAngularVelocity(int16_t* x, int16_t* y, int16_t* z) {
    *x = getAngularVelocityX();
    *y = getAngularVelocityY();
    *z = getAngularVelocityZ();
}

/** Get the angular velocity about the X-axis
 * @return Angular velocity about the X-axis
 * @see LSM9DS0_RA_OUT_X_L
 * @see LSM9DS0_RA_OUT_X_H
 */
int16_t LSM9DS0::getAngularVelocityX() {
    I2Cdev::readBytes(devAddrGyro, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_L, 2, buffer);
    return ((int16_t*) buffer)[0];
}
    
/** Get the angular velocity about the Y-axis
 * @return Angular velocity about the Y-axis
 * @see LSM9DS0_RA_OUT_Y_L
 * @see LSM9DS0_RA_OUT_Y_H
 */
int16_t LSM9DS0::getAngularVelocityY() {
    I2Cdev::readBytes(devAddrGyro, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Y_L, 2, buffer);
    return ((int16_t*) buffer)[0];
}

/** Get the angular velocity about the Z-axis
 * @return Angular velocity about the Z-axis
 * @see LSM9DS0_RA_OUT_Z_L
 * @see LSM9DS0_RA_OUT_Z_H
 */
int16_t LSM9DS0::getAngularVelocityZ() {
    I2Cdev::readBytes(devAddrGyro, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Z_L, 2, buffer);
    return ((int16_t*) buffer)[0];
}

// FIFO_CTRL register, r/w

/** Set the FIFO mode to one of the defined modes
 * @param mode New FIFO mode
 * @see LSM9DS0_RA_FIFO_CTRL
 * @see LSM9DS0_FIFO_MODE_BIT
 * @see LSM9DS0_FIFO_MODE_LENGTH
 * @see LSM9DS0_FM_BYPASS
 * @see LSM9DS0_FM_FIFO
 * @see LSM9DS0_FM_STREAM
 * @see LSM9DS0_FM_STREAM_FIFO
 * @see LSM9DS0_FM_BYPASS_STREAM
 */
void LSM9DS0::setFIFOMode(lsm9ds0_fifo_mode_t mode) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_FIFO_CTRL, LSM9DS0_FIFO_MODE_BIT, 
        LSM9DS0_FIFO_MODE_LENGTH, mode);
    I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_FIFO_CTRL_REG, LSM9DS0_FIFO_MODE_BIT, 
        LSM9DS0_FIFO_MODE_LENGTH, mode);
}

/** Get the FIFO mode to one of the defined modes
 * @return Current FIFO mode
 * @see LSM9DS0_RA_FIFO_CTRL
 * @see LSM9DS0_FIFO_MODE_BIT
 * @see LSM9DS0_FIFO_MODE_LENGTH
 * @see LSM9DS0_FM_BYPASS
 * @see LSM9DS0_FM_FIFO
 * @see LSM9DS0_FM_STREAM
 * @see LSM9DS0_FM_STREAM_FIFO
 * @see LSM9DS0_FM_BYPASS_STREAM
 */
lsm9ds0_fifo_mode_t LSM9DS0::getFIFOMode() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_FIFO_CTRL, 
        LSM9DS0_FIFO_MODE_BIT, LSM9DS0_FIFO_MODE_LENGTH, buffer);
    return (lsm9ds0_fifo_mode_t) buffer[0];
}

/** Set the FIFO watermark threshold
 * @param wtm New FIFO watermark threshold
 * @see LSM9DS0_RA_FIFO_CTRL
 * @see LSM9DS0_FIFO_WTM_BIT
 * @see LSM9DS0_FIFO_WTM_LENGTH
 */
void LSM9DS0::setGyroFIFOThreshold(uint8_t wtm) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_FIFO_CTRL, LSM9DS0_FIFO_WTM_BIT, 
        LSM9DS0_FIFO_WTM_LENGTH, wtm);
}

void LSM9DS0::setMagAccFIFOThreshold(uint8_t wtm) {
    I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_FIFO_CTRL_REG, LSM9DS0_FIFO_WTM_BIT,
        LSM9DS0_FIFO_WTM_LENGTH, wtm);
}

/** Get the FIFO watermark threshold
 * @return FIFO watermark threshold
 * @see LSM9DS0_RA_FIFO_CTRL
 * @see LSM9DS0_FIFO_WTM_BIT
 * @see LSM9DS0_FIFO_WTM_LENGTH
 */
uint8_t LSM9DS0::getGyroFIFOThreshold() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_FIFO_CTRL, LSM9DS0_FIFO_WTM_BIT,
        LSM9DS0_FIFO_WTM_LENGTH, buffer);
    return buffer[0];
}

uint8_t LSM9DS0::getMagAccFIFOThreshold() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_FIFO_CTRL, LSM9DS0_FIFO_WTM_BIT,
        LSM9DS0_FIFO_WTM_LENGTH, buffer);
    return buffer[0];
}

// FIFO_SRC register, read-only

/** Get whether the number of data sets in the FIFO buffer is less than the 
 * watermark
 * @return True if the number of data sets in the FIFO buffer is more than or 
 * equal to the watermark, false otherwise.
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_STATUS_BIT
 */
bool LSM9DS0::getGyroFIFOAtWatermark() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_FIFO_SRC, LSM9DS0_FIFO_STATUS_BIT, 
        buffer);
    return buffer[0];
}

/** Get whether the FIFO buffer is full
 * @return True if the FIFO buffer is full, false otherwise
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_OVRN_BIT
 */
bool LSM9DS0::getGyroFIFOOverrun() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_FIFO_SRC, 
        LSM9DS0_FIFO_OVRN_BIT, buffer);
    return buffer[0];
}

/** Get whether the FIFO buffer is empty
 * @return True if the FIFO buffer is empty, false otherwise
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_EMPTY_BIT
 */
bool LSM9DS0::getGyroFIFOEmpty() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_FIFO_SRC,
        LSM9DS0_FIFO_EMPTY_BIT, buffer);
    return buffer[0];
}

/** Get the number of filled FIFO buffer slots
 * @return Number of filled slots in the FIFO buffer
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_FSS_BIT
 * @see LSM9DS0_FIFO_FSS_LENGTH
 */ 
uint8_t LSM9DS0::getGyroFIFOStoredDataLevel() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_FIFO_SRC, 
        LSM9DS0_FIFO_FSS_BIT, LSM9DS0_FIFO_FSS_LENGTH, buffer);
    return buffer[0];
}

/** Get whether the number of data sets in the FIFO buffer is less than the 
 * watermark
 * @return True if the number of data sets in the FIFO buffer is more than or 
 * equal to the watermark, false otherwise.
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_STATUS_BIT
 */
bool LSM9DS0::getMagAccFIFOAtWatermark() {
    I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_FIFO_SRC_REG,
        LSM9DS0_FIFO_STATUS_BIT, buffer);
    return buffer[0];
}

/** Get whether the FIFO buffer is full
 * @return True if the FIFO buffer is full, false otherwise
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_OVRN_BIT
 */
bool LSM9DS0::getMagAccFIFOOverrun() {
    I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_FIFO_SRC_REG, 
        LSM9DS0_FIFO_OVRN_BIT, buffer);
    return buffer[0];
}

/** Get whether the FIFO buffer is empty
 * @return True if the FIFO buffer is empty, false otherwise
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_EMPTY_BIT
 */
bool LSM9DS0::getMagAccFIFOEmpty() {
    I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_FIFO_SRC_REG,
        LSM9DS0_FIFO_EMPTY_BIT, buffer);
    return buffer[0];
}

/** Get the number of filled FIFO buffer slots
 * @return Number of filled slots in the FIFO buffer
 * @see LSM9DS0_RA_FIFO_SRC
 * @see LSM9DS0_FIFO_FSS_BIT
 * @see LSM9DS0_FIFO_FSS_LENGTH
 */ 
uint8_t LSM9DS0::getMagAccFIFOStoredDataLevel() {
    I2Cdev::readBits(devAddrMagAcc, LSM9DS0_RA_FIFO_SRC_REG,
        LSM9DS0_FIFO_FSS_BIT, LSM9DS0_FIFO_FSS_LENGTH, buffer);
    return buffer[0];
}



// INT1_CFG register, r/w

/** Set the combination mode for interrupt events
 * @param combination New combination mode for interrupt events. 
 * LSM9DS0_INT1_OR for OR and LSM9DS0_INT1_AND for AND
 * @see LSM9DS0_RA_INT1_CFG
 * @see LSM9DS0_INT1_AND_OR_BIT
 * @see LSM9DS0_INT1_OR
 * @see LSM9DS0_INT1_AND
 */
void LSM9DS0::setGyroInterruptCombination(bool combination) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_INT1_AND_OR_BIT,
        combination);
}

/** Get the combination mode for interrupt events
 * @return Combination mode for interrupt events. LSM9DS0_INT1_OR for OR and 
 * LSM9DS0_INT1_AND for AND
 * @see LSM9DS0_RA_INT1_CFG
 * @see LSM9DS0_INT1_AND_OR_BIT
 * @see LSM9DS0_INT1_OR
 * @see LSM9DS0_INT1_AND
 */
bool LSM9DS0::getGyroInterruptCombination() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_INT1_AND_OR_BIT,
        buffer);
    return buffer[0];
}

/** Set whether an interrupt request is latched
 * This bit is cleared when the INT1_SRC register is read
 * @param latched New status of the latched request
 * @see LSM9DS0_RA_INT1_CFG
 * @see LSM9DS0_INT1_LIR_BIT
 */
void LSM9DS0::setGyroInterruptRequestLatched(bool latched) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_INT1_LIR_BIT, latched);
}

/** Get whether an interrupt request is latched
 * @return True if an interrupt request is latched, false otherwise
 * @see LSM9DS0_RA_INT1_CFG
 * @see LSM9DS0_INT1_LIR_BIT
 */
bool LSM9DS0::getGyroInterruptRequestLatched() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_INT1_LIR_BIT, 
        buffer); 
    return buffer[0];
};

/** Set whether the interrupt for Z high is enabled
 * @param enabled New enabled state for Z high interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_ZHIE_BIT
 */
void LSM9DS0::setGyroZHighInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_ZHIE_BIT, enabled);
}

/** Get whether the interrupt for Z high is enabled
 * @return True if the interrupt for Z high is enabled, false otherwise 
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_ZHIE_BIT
 */
bool LSM9DS0::getGyroZHighInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_ZHIE_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the interrupt for Z low is enabled
 * @param enabled New enabled state for Z low interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_ZLIE_BIT
 */
void LSM9DS0::setGyroZLowInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_ZLIE_BIT, enabled);
}

/** Get whether the interrupt for Z low is enabled
 * @return True if the interrupt for Z low is enabled, false otherwise
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_ZLIE_BIT
 */
bool LSM9DS0::getGyroZLowInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_ZLIE_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the interrupt for Y high is enabled
 * @param enabled New enabled state for Y high interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_YHIE_BIT
 */
void LSM9DS0::setGyroYHighInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_YHIE_BIT, enabled);
}

/** Get whether the interrupt for Y high is enabled
 * @return True if the interrupt for Y high is enabled, false otherwise
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_YHIE_BIT
 */
bool LSM9DS0::getGyroYHighInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_YHIE_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the interrupt for Y low is enabled
 * @param enabled New enabled state for Y low interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_YLIE_BIT
 */
void LSM9DS0::setGyroYLowInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_YLIE_BIT, enabled);
}

/** Get whether the interrupt for Y low is enabled
 * @return True if the interrupt for Y low is enabled, false otherwise 
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_YLIE_BIT
 */
bool LSM9DS0::getGyroYLowInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_YLIE_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the interrupt for X high is enabled
 * @param enabled New enabled state for X high interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_XHIE_BIT
 */
void LSM9DS0::setGyroXHighInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_XHIE_BIT, enabled);
}

/** Get whether the interrupt for X high is enabled
 * @return True if the interrupt for X high is enabled, false otherwise
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_XHIE_BIT
 */
bool LSM9DS0::getGyroXHighInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_XHIE_BIT, 
        buffer);
    return buffer[0];
}

/** Set whether the interrupt for X low is enabled
 * @param enabled New enabled state for X low interrupt.
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_XLIE_BIT
 */
void LSM9DS0::setGyroXLowInterruptEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_XLIE_BIT, enabled);
}

/** Get whether the interrupt for X low is enabled
 * @return True if the interrupt for X low is enabled, false otherwise
 * @see LSM9DS0_INT1_CFG
 * @see LSM9DS0_XLIE_BIT
 */
bool LSM9DS0::getGyroXLowInterruptEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_CFG, LSM9DS0_XLIE_BIT, 
        buffer);
    return buffer[0];
}

// INT1_SRC register, read-only

/** Get whether an interrupt has been generated
 * @return True if one or more interrupts has been generated, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_IA_BIT
 */
bool LSM9DS0::getGyroInterruptActive() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_IA_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a Z high event has occurred
 * @return True if a Z high event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_ZH_BIT
 */
bool LSM9DS0::getGyroZHigh() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_ZH_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a Z low event has occurred
 * @return True if a Z low event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_ZL_BIT
 */
bool LSM9DS0::getGyroZLow() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_ZL_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a Y high event has occurred
 * @return True if a Y high event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_YH_BIT
 */
bool LSM9DS0::getGyroYHigh() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_YH_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a Y low event has occurred
 * @return True if a Y low event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_YL_BIT
 */
bool LSM9DS0::getGyroYLow() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_YL_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a X high event has occurred
 * @return True if a X high event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_XH_BIT
 */
bool LSM9DS0::getGyroXHigh() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_XH_BIT,
        buffer);
    return buffer[0];
}

/** Get whether a X low event has occurred
 * @return True if a X low event has occurred, false otherwise
 * @see LSM9DS0_RA_INT1_SRC
 * @see LSM9DS0_INT1_XL_BIT
 */
bool LSM9DS0::getGyroXLow() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_SRC, LSM9DS0_INT1_XL_BIT,
        buffer);
    return buffer[0];
}

// INT1_THS_* registers, r/w

/** Set the threshold for a high interrupt on the X axis
 * @param threshold New threshold for a high interrupt on the X axis
 * @see LSM9DS0_INT1_THS_XH
 */
void LSM9DS0::setGyroXHighThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_XH, threshold);
}

/** Retrieve the threshold for a high interrupt on the X axis
 * @return X high interrupt threshold
 * @see LSM9DS0_INT1_THS_XH
 */
uint8_t LSM9DS0::getGyroXHighThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_XH, buffer);
    return buffer[0];
}

/** Set the threshold for a low interrupt on the X axis
 * @param threshold New threshold for a low interrupt on the X axis
 * @see LSM9DS0_INT1_THS_XL
 */
void LSM9DS0::setGyroXLowThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_XL, threshold);
}

/** Retrieve the threshold for a low interrupt on the X axis
 * @return X low interrupt threshold
 * @see LSM9DS0_INT1_THS_XL
 */
uint8_t LSM9DS0::getGyroXLowThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_XL, buffer);
    return buffer[0];
}

/** Set the threshold for a high interrupt on the Y axis
 * @param threshold New threshold for a high interrupt on the Y axis
 * @see LSM9DS0_INT1_THS_YH
 */
void LSM9DS0::setGyroYHighThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_YH, threshold);
}

/** Retrieve the threshold for a high interrupt on the Y axis
 * @return Y high interrupt threshold
 * @see LSM9DS0_INT1_THS_YH
 */
uint8_t LSM9DS0::getGyroYHighThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_YH, buffer);
    return buffer[0];
}

/** Set the threshold for a low interrupt on the Y axis
 * @param threshold New threshold for a low interrupt on the Y axis
 * @see LSM9DS0_INT1_THS_YL
 */
void LSM9DS0::setGyroYLowThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_YL, threshold);
}

/** Retrieve the threshold for a low interrupt on the Y axis
 * @return Y low interrupt threshold
 * @see LSM9DS0_INT1_THS_YL
 */
uint8_t LSM9DS0::getGyroYLowThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_YL, buffer);
    return buffer[0];
}

/** Set the threshold for a high interrupt on the Z axis
 * @param threshold New threshold for a high interrupt on the Z axis
 * @see LSM9DS0_INT1_THS_ZH
 */
void LSM9DS0::setGyroZHighThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_ZH, threshold);
}

/** Retrieve the threshold for a high interrupt on the Z axis
 * @return Z high interrupt threshold
 * @see LSM9DS0_INT1_THS_ZH
 */
uint8_t LSM9DS0::getGyroZHighThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_ZH, buffer);
    return buffer[0];
}

/** Set the threshold for a low interrupt on the Z axis
 * @param threshold New threshold for a low interrupt on the Z axis
 * @see LSM9DS0_RA_INT1_THS_ZL
 */
void LSM9DS0::setGyroZLowThreshold(uint8_t threshold) {
    I2Cdev::writeByte(devAddrGyro, LSM9DS0_RA_INT1_THS_ZL, threshold);
}

/** Retrieve the threshold for a low interrupt on the Z axis
 * @return Z low interrupt threshold
 * @see LSM9DS0_INT1_THS_ZL
 */
uint8_t LSM9DS0::getGyroZLowThreshold() {
    I2Cdev::readByte(devAddrGyro, LSM9DS0_RA_INT1_THS_ZL, buffer);
    return buffer[0];
}

// INT1_DURATION register, r/w

/* Set the minimum duration for an interrupt event to be recognized
 * This depends on the chosen output data rate
 * @param duration New duration necessary for an interrupt event to be 
 * recognized
 * @see LSM9DS0_RA_INT1_DURATION
 * @see LSM9DS0_INT1_DUR_BIT
 * @see LSM9DS0_INT1_DUR_LENGTH
 */
void LSM9DS0::setGyroDuration(uint8_t duration) {
    I2Cdev::writeBits(devAddrGyro, LSM9DS0_RA_INT1_DURATION, LSM9DS0_INT1_DUR_BIT,
        LSM9DS0_INT1_DUR_LENGTH, duration);
}

/** Get the minimum duration for an interrupt event to be recognized
 * @return Duration necessary for an interrupt event to be recognized
 * @see LSM9DS0_RA_INT1_DURATION
 * @see LSM9DS0_INT1_DUR_BIT
 * @see LSM9DS0_INT1_DUR_LENGTH
 */
uint8_t LSM9DS0::getGyroDuration() {
    I2Cdev::readBits(devAddrGyro, LSM9DS0_RA_INT1_DURATION, 
        LSM9DS0_INT1_DUR_BIT, LSM9DS0_INT1_DUR_LENGTH, buffer);
    return buffer[0];
}

/** Set whether the interrupt wait feature is enabled
 * If false, the interrupt falls immediately if signal crosses the selected 
 * threshold. Otherwise, if signal crosses the selected threshold, the interrupt
 * falls only after the duration has counted number of samples at the selected 
 * data rate, written into the duration counter register.
 * @param enabled New enabled state of the interrupt wait
 * @see LSM9DS0_RA_INT1_DURATION
 * @see LSM9DS0_INT1_WAIT_BIT
 */
void LSM9DS0::setGyroWaitEnabled(bool enabled) {
    I2Cdev::writeBit(devAddrGyro, LSM9DS0_RA_INT1_DURATION, LSM9DS0_INT1_WAIT_BIT,
        enabled);
}

/** Get whether the interrupt wait feature is enabled
 * @return True if the wait feature is enabled, false otherwise
 * @see LSM9DS0_RA_INT1_DURATION
 * @see LSM9DS0_INT1_WAIT_BIT
 */
bool LSM9DS0::getGyroWaitEnabled() {
    I2Cdev::readBit(devAddrGyro, LSM9DS0_RA_INT1_DURATION, 
        LSM9DS0_INT1_WAIT_BIT, buffer);
    return buffer[0];
}

/** Get the current temperature, sampled with the same rate
 * as the compass. According to the datasheet this has an resolution
 * of 8 LSB/°C.
 * @see LSM9DS0_RA_OUT_TEMP_XM
 */
int16_t LSM9DS0::getTemperature() {
  int16_t temperature = 0;
  I2Cdev::readWord(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_TEMP_XM, (uint16_t*) &temperature);
  return temperature;
}

float LSM9DS0::getTemperatureCelsius() {
  return getTemperature() / 800.;
}

bool LSM9DS0::getMagXYZOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_ZYXOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagZOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_ZOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagYOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_YOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagXOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_XOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagXYZDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_ZYXDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagZDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_ZDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagYDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_YDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getMagXDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_M,
      LSM9DS0_XDA_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::getMagneticFieldStrength(int16_t* x, int16_t* y, int16_t* z) {
  *x = getMagneticFieldStrengthX();
  *y = getMagneticFieldStrengthY();
  *z = getMagneticFieldStrengthZ();
}

int16_t LSM9DS0::getMagneticFieldStrengthX() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t LSM9DS0::getMagneticFieldStrengthY() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Y_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

int16_t LSM9DS0::getMagneticFieldStrengthZ() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Z_M, 2, buffer);
  return (((int16_t)buffer[0]) << 8) | buffer[1];
}

bool LSM9DS0::getMagXAxisInterrupt() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_XMIEN_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setMagXAxisInterrupt(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_XMIEN_BIT, enable);
}

bool LSM9DS0::getMagYAxisInterrupt() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_YMIEN_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setMagYAxisInterrupt(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_YMIEN_BIT, enable);
}

bool LSM9DS0::getMagZAxisInterrupt() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_ZMIEN_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setMagZAxisInterrupt(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_ZMIEN_BIT, enable);
}

bool LSM9DS0::getMagAccInterruptOpenDrain() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_PP_OD_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setMagAccInterruptOpenDrain(bool isopendrain) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_PP_OD_BIT, isopendrain);
}

bool LSM9DS0::getInterruptPolarity() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_IEA_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::getInterruptPolarity(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_IEA_BIT, enable);
}

bool LSM9DS0::getMagAccInterruptLatch() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_IEL_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::getMagAccInterruptLatch(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_IEL_BIT, enable);
}
bool LSM9DS0::getAcc4DDetection() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_4D_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setAcc4DDetection(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_4D_BIT, enable);
}

bool LSM9DS0::getMagInterruptEnable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_MIEN_BIT, buffer);
  return buffer[0];
}

void LSM9DS0::setMagInterruptEnable(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_INT_CTRL_REG_M, LSM9DS0_MIEN_BIT, enable);
}

void LSM9DS0::setMagAccWatermarkEnable(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_WTM_EN_BIT, enable);
}

void LSM9DS0::setMagAccHighPassForClick(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_HP_CLICK_BIT, enable);
}

void LSM9DS0::setMagAccHighPassIrq1(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_HPIS1_BIT, enable);
}

void LSM9DS0::setMagAccHighPassIrq2(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG0_XM, LSM9DS0_XM_HPIS2_BIT, enable);
}

void LSM9DS0::setAccRate(lsm9ds0_acc_data_rate_t rate) {
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG1_XM, LSM9DS0_XM_AODR_BIT,
      LSM9DS0_XM_AODR_LENGTH, rate);
}

void LSM9DS0::setAccAntiAliasFilterBandwidth(lsm9ds0_acc_filter_bw_t bw) {
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG2_XM, LSM9DS0_XM_ABW_BIT,
      LSM9DS0_XM_ABW_LENGTH, bw);
}

void LSM9DS0::setAccFullScale(lsm9ds0_acc_full_scale_t scale) {
  switch (scale) {
    case LSM9DS0_ACC_2G:
      acc_scale = 0.061 / 1000. * 9.81;
      break;
    case LSM9DS0_ACC_4G:
      acc_scale = 0.122 / 1000. * 9.81;
      break;
    case LSM9DS0_ACC_6G:
      acc_scale = 0.183 / 1000. * 9.81;
      break;
    case LSM9DS0_ACC_8G:
      acc_scale = 0.244 / 1000. * 9.81;
      break;
    case LSM9DS0_ACC_16G:
      acc_scale = 0.732 / 1000. * 9.81;
      break;
    }
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG2_XM, LSM9DS0_XM_AFS_BIT,
      LSM9DS0_XM_AFS_LENGTH, scale);
}

lsm9ds0_acc_full_scale_t LSM9DS0::getAccFullScale() {
  I2Cdev::readBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG2_XM, LSM9DS0_XM_AFS_BIT,
      LSM9DS0_XM_AFS_LENGTH, buffer);
  return (lsm9ds0_acc_full_scale_t) buffer[0];
}

void LSM9DS0::setMagResolution(bool high) {
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG5_XM, LSM9DS0_XM_M_RES_BIT,
      LSM9DS0_XM_M_RES_LENGTH, high ? 0b11 : 0b00);
}

void LSM9DS0::setMagOutputRate(lsm9ds0_mag_output_rate_t rate) {
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG5_XM, LSM9DS0_XM_M_ODR_LENGTH,
      LSM9DS0_XM_M_ODR_LENGTH, rate);
}

void LSM9DS0::setMagFullScale(lsm9ds0_mag_scale_t scale) {
  switch (scale) {
    case LSM9DS0_MAG_2_GAUSS:
      mag_scale=0.08 / 1000.;
      break;
    case LSM9DS0_MAG_4_GAUSS:
      mag_scale=0.16 / 1000.;
      break;
    case LSM9DS0_MAG_8_GAUSS:
      mag_scale=0.32 / 1000.;
      break;
    case LSM9DS0_MAG_12_GAUSS:
      mag_scale=0.48 / 1000.;
      break;
  }
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG6_XM, LSM9DS0_XM_MFS_BIT,
      LSM9DS0_XM_MFS_LENGTH, scale);
}

lsm9ds0_mag_scale_t LSM9DS0::getMagFullScale() {
  I2Cdev::readBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG6_XM, LSM9DS0_XM_MFS_BIT,
      LSM9DS0_XM_MFS_LENGTH, buffer);
  return (lsm9ds0_mag_scale_t) buffer[0];
}

void LSM9DS0::setAccHighPassFilterMode(lsm9ds0_acc_highpass_fitermode_t fm) {
  I2Cdev::writeBits(devAddrMagAcc, LSM9DS0_RA_CTRL_REG7_XM, LSM9DS0_XM_AHPDM_BIT,
      LSM9DS0_XM_AHPDM_LENGTH, fm);
}

void LSM9DS0::setFilterEnable(bool enable) {
  I2Cdev::writeBit(devAddrMagAcc, LSM9DS0_RA_CTRL_REG7_XM, LSM9DS0_XM_AFDS_BIT, enable);
}

void LSM9DS0::getAccleration(int16_t* x, int16_t* y, int16_t* z) {
  *x = getAcclerationX();
  *y = getAcclerationY();
  *z = getAcclerationZ();
}

bool LSM9DS0::getAccXYZOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_ZYXOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccZOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_ZOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccYOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_YOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccXOverrun() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_XOR_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccXYZDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_ZYXDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccZDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_ZDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccYDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_YDA_BIT, buffer);
  return buffer[0];
}

bool LSM9DS0::getAccXDataAvailable() {
  I2Cdev::readBit(devAddrMagAcc, LSM9DS0_RA_STATUS_REG_A,
      LSM9DS0_XDA_BIT, buffer);
  return buffer[0];
}

int16_t LSM9DS0::getAcclerationX() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_A, 2, buffer);
  return ((int16_t*) buffer)[0];
}

int16_t LSM9DS0::getAcclerationY() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Y_A, 2, buffer);
  return ((int16_t*) buffer)[0];
}

int16_t LSM9DS0::getAcclerationZ() {
  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_Z_A, 2, buffer);
  return ((int16_t*) buffer)[0];
}

void LSM9DS0::updateMeasurement(lsm9d_measurement_t *m) {
  int16_t *val = (int16_t*) buffer;

  /* this blocks the function for the given sampling rate */
  while( !getGyroXYZDataAvailable() )
    ;

  I2Cdev::readBytes(devAddrGyro, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_L, 6, buffer);
  m->gx = gyr_scale * val[0] * M_PI/180.;
  m->gy = gyr_scale * val[1] * M_PI/180.;
  m->gz = gyr_scale * val[2] * M_PI/180.;

  /* reading the accelerometer and magnetometer is done only if neccesary */
  while( getAccXYZDataAvailable() ) {
    I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_A, 6, buffer);
    m->ax = acc_scale * val[0];
    m->ay = acc_scale * val[1];
    m->az = acc_scale * val[2];
  }

  while( getMagXYZDataAvailable() ) {
    I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_M, 6, buffer);
    m->mx = mag_scale * val[0];
    m->my = mag_scale * val[1];
    m->mz = mag_scale * val[2];
  }

}

lsm9d_measurement_t LSM9DS0::getMeasurement() {
  lsm9d_measurement_t m;
  int16_t *val = (int16_t*) buffer;

  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_A, 6, buffer);
  m.ax = acc_scale * val[0];
  m.ay = acc_scale * val[1];
  m.az = acc_scale * val[2];

  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_M, 6, buffer);
  m.mx = mag_scale * val[0];
  m.my = mag_scale * val[1];
  m.mz = mag_scale * val[2];

  I2Cdev::readBytes(devAddrGyro, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_X_L, 6, buffer);
  m.gx = gyr_scale * val[0] * M_PI/180.;
  m.gy = gyr_scale * val[1] * M_PI/180.;
  m.gz = gyr_scale * val[2] * M_PI/180.;

  I2Cdev::readBytes(devAddrMagAcc, BIT_AUTOINCREMENT|LSM9DS0_RA_OUT_TEMP, 6, buffer);
  m.t = val[0] / 800.;

  return m;
}
