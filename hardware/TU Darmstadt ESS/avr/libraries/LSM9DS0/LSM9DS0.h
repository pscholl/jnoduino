// I2Cdev library collection - MPL115A2 I2C device class header file
// Based on Freescale MPL115A2 datasheet, [02/2013]
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

#ifndef _LSM9DS0_H_
#define _LSM9DS0_H_

#include "I2Cdev.h"

#define LSM9DS0_ACCMAG_ADDRESS_AD0_LOW   0x1E
#define LSM9DS0_ACCMAG_ADDRESS_AD0_HIGH  0x1D
#define LSM9DS0_ACCMAG_DEFAULT_ADDRESS   LSM9DS0_ACCMAG_ADDRESS_AD0_LOW

#define LSM9DS0_GYRO_ADDRESS_AD0_LOW   0x6A
#define LSM9DS0_GYRO_ADDRESS_AD0_HIGH  0x6B
#define LSM9DS0_GYRO_DEFAULT_ADDRESS   LSM9DS0_GYRO_ADDRESS_AD0_LOW

#define BIT_AUTOINCREMENT (1<<7)

typedef enum {
  LSM9DS0_RA_WHO_AM_I  = 0x0F,
  LSM9DS0_RA_CTRL_REG1 = 0x20,
  LSM9DS0_RA_CTRL_REG2,
  LSM9DS0_RA_CTRL_REG3,
  LSM9DS0_RA_CTRL_REG4,
  LSM9DS0_RA_CTRL_REG5,
  LSM9DS0_RA_REFERENCE,
  LSM9DS0_RA_OUT_TEMP,
  LSM9DS0_RA_STATUS,
  LSM9DS0_RA_OUT_X_L,
  LSM9DS0_RA_OUT_X_H,
  LSM9DS0_RA_OUT_Y_L,
  LSM9DS0_RA_OUT_Y_H,
  LSM9DS0_RA_OUT_Z_L,
  LSM9DS0_RA_OUT_Z_H,
  LSM9DS0_RA_FIFO_CTRL,
  LSM9DS0_RA_FIFO_SRC,
  LSM9DS0_RA_INT1_CFG,
  LSM9DS0_RA_INT1_SRC,
  LSM9DS0_RA_INT1_THS_XH,
  LSM9DS0_RA_INT1_THS_XL,
  LSM9DS0_RA_INT1_THS_YH,
  LSM9DS0_RA_INT1_THS_YL,
  LSM9DS0_RA_INT1_THS_ZH,
  LSM9DS0_RA_INT1_THS_ZL,
  LSM9DS0_RA_INT1_DURATION,
} lsmd9s0_gyro_regs;

typedef enum {
  LSM9DS0_RA_OUT_TEMP_XM  = 0x05,
  LSM9DS0_RA_STATUS_REG_M = 0x07,
  LSM9DS0_RA_OUT_X_M      = 0x08,
  LSM9DS0_RA_OUT_Y_M      = 0x0A,
  LSM9DS0_RA_OUT_Z_M      = 0x0C,
  LSM9DS0_RA_WHO_AM_I_M   = 0x0F,
  LSM9DS0_RA_INT_CTRL_REG_M = 0x12,
  LSM9DS0_RA_INT_SRC_REG_M  = 0x13,
  LSM9DS0_RA_INT_THS_L      = 0x14,
  LSM9DS0_RA_OFFSET_X_M = 0x16,
  LSM9DS0_RA_OFFSET_Y_M = 0x18,
  LSM9DS0_RA_OFFSET_Z_M = 0x1A,
  LSM9DS0_RA_REFERENCE_X = 0x1C,
  LSM9DS0_RA_REFERENCE_Y,
  LSM9DS0_RA_REFERENCE_Z,
  LSM9DS0_RA_CTRL_REG0_XM,
  LSM9DS0_RA_CTRL_REG1_XM,
  LSM9DS0_RA_CTRL_REG2_XM,
  LSM9DS0_RA_CTRL_REG3_XM,
  LSM9DS0_RA_CTRL_REG4_XM,
  LSM9DS0_RA_CTRL_REG5_XM,
  LSM9DS0_RA_CTRL_REG6_XM,
  LSM9DS0_RA_CTRL_REG7_XM,
  LSM9DS0_RA_STATUS_REG_A = 0x27,
  LSM9DS0_RA_OUT_X_A,
  LSM9DS0_RA_OUT_Y_A = 0x2A,
  LSM9DS0_RA_OUT_Z_A = 0x2C,
  LSM9DS0_RA_FIFO_CTRL_REG = 0x2E,
  LSM9DS0_RA_FIFO_SRC_REG,
  LSM9DS0_RA_INT_GEN_1_REG,
  LSM9DS0_RA_INT_GEN_1_SRC,
  LSM9DS0_RA_INT_GEN_1_THS,
  LSM9DS0_RA_INT_GEN_1_DURATION,
  LSM9DS0_RA_INT_GEN_2_REG,
  LSM9DS0_RA_INT_GEN_2_SRC,
  LSM9DS0_RA_INT_GEN_2_THS,
  LSM9DS0_RA_INT_GEN_2_DURATION,
  LSM9DS0_RA_CLICK_CFG,
  LSM9DS0_RA_CLICK_THS,
  LSM9DS0_RA_TIME_LATENCY,
  LSM9DS0_RA_TIME_WINDOW,
  LSM9DS0_RA_ACT_THS,
  LSM9DS0_RA_ACT_DUR
} lsmd9s0_accmag_regs;

#define LSM9DS0_ODR_BIT           7
#define LSM9DS0_ODR_LENGTH        2
#define LSM9DS0_BW_BIT            5
#define LSM9DS0_BW_LENGTH         2
#define LSM9DS0_PD_BIT            3
#define LSM9DS0_ZEN_BIT           2
#define LSM9DS0_YEN_BIT           1
#define LSM9DS0_XEN_BIT           0

typedef enum {
  LSM9DS0_RATE_95           = 0b00,
  LSM9DS0_RATE_190          = 0b01,
  LSM9DS0_RATE_380          = 0b10,
  LSM9DS0_RATE_760          = 0b11
} gyro_rate_t;

typedef enum {
  LSM9DS0_BW_LOW            = 0b00,
  LSM9DS0_BW_MED_LOW        = 0b01,
  LSM9DS0_BW_MED_HIGH       = 0b10,
  LSM9DS0_BW_HIGH           = 0b11
} gyro_bandwidth_t;

#define LSM9DS0_HPM_BIT           5
#define LSM9DS0_HPM_LENGTH        2
#define LSM9DS0_HPCF_BIT          3
#define LSM9DS0_HPCF_LENGTH       4

typedef enum {
  LSM9DS0_HPM_HRF           = 0b00,
  LSM9DS0_HPM_REFERENCE     = 0b01,
  LSM9DS0_HPM_NORMAL        = 0b10,
  LSM9DS0_HPM_AUTORESET     = 0b11
} lsm9ds0_acc_highpass_fitermode_t;

#define LSM9DS0_HPCF1             0b0000
#define LSM9DS0_HPCF2             0b0001
#define LSM9DS0_HPCF3             0b0010
#define LSM9DS0_HPCF4             0b0011
#define LSM9DS0_HPCF5             0b0100
#define LSM9DS0_HPCF6             0b0101
#define LSM9DS0_HPCF7             0b0110
#define LSM9DS0_HPCF8             0b0111
#define LSM9DS0_HPCF9             0b1000
#define LSM9DS0_HPCF10            0b1001

#define LSM9DS0_I1_INT1_BIT       7
#define LSM9DS0_I1_BOOT_BIT       6
#define LSM9DS0_H_LACTIVE_BIT     5
#define LSM9DS0_PP_OD_BIT         4
#define LSM9DS0_I2_DRDY_BIT       3
#define LSM9DS0_I2_WTM_BIT        2
#define LSM9DS0_I2_ORUN_BIT       1
#define LSM9DS0_I2_EMPTY_BIT      0

#define LSM9DS0_PUSH_PULL         1
#define LSM9DS0_OPEN_DRAIN        0

#define LSM9DS0_BDU_BIT           7
#define LSM9DS0_BLE_BIT           6
#define LSM9DS0_FS_BIT            5
#define LSM9DS0_FS_LENGTH         2
#define LSM9DS0_ST_BIT            2
#define LSM9DS0_ST_LENGTH         2
#define LSM9DS0_SIM_BIT           0

#define LSM9DS0_BIG_ENDIAN        1
#define LSM9DS0_LITTLE_ENDIAN     0

#define LSM9DS0_FS_250            0b00
#define LSM9DS0_FS_500            0b01
#define LSM9DS0_FS_2000           0b10

#define LSM9DS0_SELF_TEST_NORMAL  0b00
#define LSM9DS0_SELF_TEST_0       0b01
#define LSM9DS0_SELF_TEST_1       0b11

#define LSM9DS0_SPI_4_WIRE        1
#define LSM9DS0_SPI_3_WIRE        0

#define LSM9DS0_BOOT_BIT          7
#define LSM9DS0_FIFO_EN_BIT       6
#define LSM9DS0_HPEN_BIT          4
#define LSM9DS0_INT1_SEL_BIT      3
#define LSM9DS0_INT1_SEL_LENGTH   2
#define LSM9DS0_OUT_SEL_BIT       1
#define LSM9DS0_OUT_SEL_LENGTH    2

#define LSM9DS0_NON_HIGH_PASS     0b00
#define LSM9DS0_HIGH_PASS         0b01
#define LSM9DS0_LOW_PASS          0b10
#define LSM9DS0_LOW_HIGH_PASS     0b11

#define LSM9DS0_ZYXOR_BIT         7
#define LSM9DS0_ZOR_BIT           6
#define LSM9DS0_YOR_BIT           5
#define LSM9DS0_XOR_BIT           4
#define LSM9DS0_ZYXDA_BIT         3
#define LSM9DS0_ZDA_BIT           2
#define LSM9DS0_YDA_BIT           1
#define LSM9DS0_XDA_BIT           0

#define LSM9DS0_FIFO_MODE_BIT     7
#define LSM9DS0_FIFO_MODE_LENGTH  3
#define LSM9DS0_FIFO_WTM_BIT      4
#define LSM9DS0_FIFO_WTM_LENGTH   5

typedef enum {
  LSM9DS0_FM_BYPASS         = 0b000,
  LSM9DS0_FM_FIFO           = 0b001,
  LSM9DS0_FM_STREAM         = 0b010,
  LSM9DS0_FM_STREAM_FIFO    = 0b011,
  LSM9DS0_FM_BYPASS_STREAM  = 0b100
} lsm9ds0_fifo_mode_t;

#define LSM9DS0_FIFO_STATUS_BIT   7
#define LSM9DS0_FIFO_OVRN_BIT     6
#define LSM9DS0_FIFO_EMPTY_BIT    5
#define LSM9DS0_FIFO_FSS_BIT      4
#define LSM9DS0_FIFO_FSS_LENGTH   5

#define LSM9DS0_INT1_AND_OR_BIT   7
#define LSM9DS0_INT1_LIR_BIT      6
#define LSM9DS0_ZHIE_BIT          5
#define LSM9DS0_ZLIE_BIT          4
#define LSM9DS0_YHIE_BIT          3
#define LSM9DS0_YLIE_BIT          2
#define LSM9DS0_XHIE_BIT          1
#define LSM9DS0_XLIE_BIT          0

#define LSM9DS0_INT1_OR           0
#define LSM9DS0_INT1_AND          1

#define LSM9DS0_INT1_IA_BIT       6
#define LSM9DS0_INT1_ZH_BIT       5
#define LSM9DS0_INT1_ZL_BIT       4
#define LSM9DS0_INT1_YH_BIT       3
#define LSM9DS0_INT1_YL_BIT       2
#define LSM9DS0_INT1_XH_BIT       1
#define LSM9DS0_INT1_XL_BIT       0

#define LSM9DS0_INT1_WAIT_BIT     7
#define LSM9DS0_INT1_DUR_BIT      6
#define LSM9DS0_INT1_DUR_LENGTH   7

#define LSM9DS0_XMIEN_BIT         7
#define LSM9DS0_YMIEN_BIT         6
#define LSM9DS0_ZMIEN_BIT         5
#define LSM9DS0_PP_OD_BIT         4
#define LSM9DS0_IEA_BIT           3
#define LSM9DS0_IEL_BIT           2
#define LSM9DS0_4D_BIT            1
#define LSM9DS0_MIEN_BIT          0

#define LSM9DS0_M_PTH_X_BIT       7
#define LSM9DS0_M_PTH_Y_BIT       6
#define LSM9DS0_M_PTH_Z_BIT       5
#define LSM9DS0_M_NTH_X_BIT       4
#define LSM9DS0_M_NTH_Y_BIT       3
#define LSM9DS0_M_NTH_Z_BIT       2
#define LSM9DS0_M_MROI_BIT        1
#define LSM9DS0_M_MINT_BIT        0

#define LSM9DS0_XM_BOOT_BIT       7
#define LSM9DS0_XM_FIFO_EN_BIT    6
#define LSM9DS0_XM_WTM_EN_BIT     5
#define LSM9DS0_XM_HP_CLICK_BIT   3
#define LSM9DS0_XM_HPIS1_BIT      1
#define LSM9DS0_XM_HPIS2_BIT      0

#define LSM9DS0_XM_AODR_BIT       7
#define LSM9DS0_XM_AODR_LENGTH    4
#define LSM9DS0_XM_BDU_BIT        3
#define LSM9DS0_XM_AZEN           2
#define LSM9DS0_XM_AYEN           1
#define LSM9DS0_XM_AXEN           0

typedef enum {
  LSM9DS0_ACC_RATE_PDOWN    = 0b0000,
  LSM9DS0_ACC_RATE_3_125    = 0b0001,
  LSM9DS0_ACC_RATE_6_25     = 0b0010,
  LSM9DS0_ACC_RATE_12_5     = 0b0011,
  LSM9DS0_ACC_RATE_25       = 0b0100,
  LSM9DS0_ACC_RATE_50       = 0b0101,
  LSM9DS0_ACC_RATE_100      = 0b0110,
  LSM9DS0_ACC_RATE_200      = 0b0111,
  LSM9DS0_ACC_RATE_400      = 0b1000,
  LSM9DS0_ACC_RATE_800      = 0b1001,
  LSM9DS0_ACC_RATE_1600     = 0b1010,
} lsm9ds0_acc_data_rate_t;

#define LSM9DS0_XM_ABW_BIT        7
#define LSM9DS0_XM_ABW_LENGTH     2
#define LSM9DS0_XM_AFS_BIT        5
#define LSM9DS0_XM_AFS_LENGTH     3
#define LSM9DS0_XM_AST_BIT        2
#define LSM9DS0_XM_AST_LENGTH     2
#define LSM9DS0_XM_SIM_BIT        0

typedef enum {
  LSM9DS0_ACC_FILTER_BW_773 = 0b00,
  LSM9DS0_ACC_FILTER_BW_194 = 0b01,
  LSM9DS0_ACC_FILTER_BW_362 = 0b10,
  LSM9DS0_ACC_FILTER_BW_50  = 0b11,
} lsm9ds0_acc_filter_bw_t;

typedef enum {
  LSM9DS0_ACC_2G   = 0b000,
  LSM9DS0_ACC_4G  = 0b001,
  LSM9DS0_ACC_6G  = 0b010,
  LSM9DS0_ACC_8G  = 0b011,
  LSM9DS0_ACC_16G = 0b100,
} lsm9ds0_acc_full_scale_t;

#define LSM9DS0_ACC_ST_NORMAL      0b00
#define LSM9DS0_ACC_ST_POSSIGN     0b01
#define LSM9DS0_ACC_ST_NEGSIGN     0b10

#define LSM9DS0_XM_P1_BOOT_BIT   7
#define LSM9DS0_XM_P1_TAP_BIT    6
#define LSM9DS0_XM_P1_INT1_BIT   5
#define LSM9DS0_XM_P1_INT2_BIT   4
#define LSM9DS0_XM_P1_INTM_BIT   3
#define LSM9DS0_XM_P1_DRDYA_BIT  2
#define LSM9DS0_XM_P1_DRDYM_BIT  1
#define LSM9DS0_XM_P1_EMPTY_BIT  0

#define LSM9DS0_XM_P2_TAP_BIT    7
#define LSM9DS0_XM_P2_INT1_BIT   6
#define LSM9DS0_XM_P2_INT2_BIT   5
#define LSM9DS0_XM_P2_INTM_BIT   4
#define LSM9DS0_XM_P2_DRDYA_BIT  3
#define LSM9DS0_XM_P2_DRDYM_BIT  2
#define LSM9DS0_XM_P2_OVRUN_BIT  1
#define LSM9DS0_XM_P2_WTM_BIT    0

#define LSM9DS0_XM_TEMP_EN_BIT   7
#define LSM9DS0_XM_M_RES_BIT     6
#define LSM9DS0_XM_M_RES_LENGTH  2
#define LSM9DS0_XM_M_ODR_BIT     4
#define LSM9DS0_XM_M_ODR_LENGTH  3
#define LSM9DS0_XM_M_LIR1        1
#define LSM9DS0_XM_M_LIR2        0

typedef enum {
  LSM9DS0_M_ODR_3_125      = 0b000,
  LSM9DS0_M_ODR_6_25       = 0b001,
  LSM9DS0_M_ODR_12_5       = 0b010,
  LSM9DS0_M_ODR_25         = 0b011,
  LSM9DS0_M_ODR_50         = 0b100,
  LSM9DS0_M_ODR_100        = 0b101
} lsm9ds0_mag_output_rate_t;

#define LSM9DS0_XM_MFS_BIT       6
#define LSM9DS0_XM_MFS_LENGTH    2

typedef enum {
  LSM9DS0_MAG_2_GAUSS        = 0b00,
  LSM9DS0_MAG_4_GAUSS        = 0b01,
  LSM9DS0_MAG_8_GAUSS        = 0b10,
  LSM9DS0_MAG_12_GAUSS       = 0b11
} lsm9ds0_mag_scale_t;

#define LSM9DS0_XM_AHPDM_BIT     7
#define LSM9DS0_XM_AHPDM_LENGTH  2
#define LSM9DS0_XM_AFDS_BIT      5
#define LSM9DS0_XM_MLP_BIT       2
#define LSM9DS0_XM_MD_BIT        1
#define LSM9DS0_XM_MD_LENGTH     0

#define LSM9DS0_ACC_HPF_NORMAL    0b00
#define LSM9DS0_ACC_HPF_REF       0b01
#define LSM9DS0_ACC_HPF_AUTORESET 0b11

#define LSM9DS0_M_MODE_CONT      0b00
#define LSM9DS0_M_MODE_SINGLE    0b01
#define LSM9DS0_M_MODE_PDWN      0b10

#define LSM9DS0_A_ZYX_OVRUN_BIT  7
#define LSM9DS0_A_Z_OVRUN_BIT    6
#define LSM9DS0_A_Y_OVRUN_BIT    5
#define LSM9DS0_A_X_OVRUN_BIT    4
#define LSM9DS0_A_ZYX_DRDY_BIT   3
#define LSM9DS0_A_Z_DRDY_BIT     2
#define LSM9DS0_A_Y_DRDY_BIT     1
#define LSM9DS0_A_X_DRDY_BIT     0

#define LSM9DS0_XM_FIFO_MODE_BIT 7
#define LSM9DS0_XM_FIFO_MODE_LENGTH 3
#define LSM9DS0_XM_FIFO_TRSH_BIT 4
#define LSM9DS0_XM_FIFO_TRSH_LENGTH 5

#define LSM9DS0_XM_WTM_BIT       7
#define LSM9DS0_XM_OVRUN_BIT     6
#define LSM9DS0_XM_EMPTY_BIT     5
#define LSM9DS0_XM_FSS_BIT       4
#define LSM9DS0_XM_FSS_LENGTH    5

#define LSM9DS0_INT1_AOI_BIT     7

typedef struct {
  unsigned long timestamp;
  float gx,gy,gz, // in rad/s
        mx,my,mz, // in gauss
        ax,ay,az, // in m/s²
        t;        // in °Celsius
} lsm9d_measurement_t;

class LSM9DS0 {
    public:
        LSM9DS0();
        LSM9DS0(uint8_t address, uint8_t accmag);

        void initialize();
        bool testConnection();

        lsm9d_measurement_t getMeasurement();
        void updateMeasurement(lsm9d_measurement_t *m);

        // WHO_AM_I register, read-only
        uint8_t getDeviceID();
        uint8_t getDeviceIDMagAcc();

        // CTRL_REG1 register, r/w
        void        setGyroOutputDataRate(gyro_rate_t rate);
        gyro_rate_t getGyroOutputDataRate();
        void             setGyroBandwidthCutOffMode(gyro_bandwidth_t mode);
        gyro_bandwidth_t getGyroBandwidthCutOffMode();
        float            getGyroBandwidthCutOff();
        void setGyroPowerOn(bool on);
        bool getGyroPowerOn();
        void setGyroZEnabled(bool enabled);
        bool getGyroZEnabled();
        void setGyroYEnabled(bool enabled);
        bool getGyroYEnabled();
        void setGyroXEnabled(bool enabled);
        bool getGyroXEnabled();

        // CTRL_REG2 register, r/w
        void setGyroHighPassMode(uint8_t mode);
        uint8_t getGyroHighPassMode();
        void setGyroHighPassFilterCutOffFrequencyLevel(uint8_t level);
        uint8_t getGyroHighPassFilterCutOffFrequencyLevel();

        // CTRL_REG3 register, r/w
        void setGyroINT1InterruptEnabled(bool enabled);
        bool getGyroINT1InterruptEnabled();
        void setGyroINT1BootStatusEnabled(bool enabled);
        bool getGyroINT1BootStatusEnabled();
        void interruptActiveINT1Config();
        void setGyroOutputMode(bool mode);
        bool getGyroOutputMode();
        void setGyroINT2DataReadyEnabled(bool enabled);
        bool getGyroINT2DataReadyEnabled();
        void setGyroINT2FIFOWatermarkInterruptEnabled(bool enabled);
        bool getGyroINT2FIFOWatermarkInterruptEnabled();
        void setGyroINT2FIFOOverrunInterruptEnabled(bool enabled);
        bool getGyroINT2FIFOOverrunInterruptEnabled();
        void setGyroINT2FIFOEmptyInterruptEnabled(bool enabled);
        bool getGyroINT2FIFOEmptyInterruptEnabled();

        // CTRL_REG4 register, r/w
        void setGyroBlockDataUpdateEnabled(bool enabled);
        bool getGyroBlockDataUpdateEnabled();
        void setGyroEndianMode(bool endianness);
        bool getGyroEndianMode();
        void setGyroFullScale(uint16_t scale);
        uint16_t getGyroFullScale();
        void setGyroSelfTestMode(uint8_t mode);
        uint8_t getGyroSelfTestMode();
        void setGyroSPIMode(bool mode);
        bool getGyroSPIMode();

        // CTRL_REG5 and CTRL_REG0_XM register, r/w
        void rebootMemoryContent();
        void setFIFOEnabled(bool enabled);
        bool getFIFOEnabled();
        void setGyroHighPassFilterEnabled(bool enabled);
        bool getGyroHighPassFilterEnabled();
        void setGyroDataFilter(uint8_t filter);
        uint8_t getGyroDataFilter();

        // REFERENCE/DATACAPTURE register, r/w
        void setGyroInterruptReference(uint8_t reference);
        uint8_t getGyroInterruptReference();

        // STATUS register, read-only
        bool getGyroXYZOverrun();
        bool getGyroZOverrun();
        bool getGyroYOverrun();
        bool getGyroXOverrun();
        bool getGyroXYZDataAvailable();
        bool getGyroZDataAvailable();
        bool getGyroYDataAvailable();
        bool getGyroXDataAvailable();

        // OUT_* registers, read-only
        void    getAngularVelocity(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAngularVelocityX();
        int16_t getAngularVelocityY();
        int16_t getAngularVelocityZ();

        // FIFO_CTRL register, r/w, for both Gyro and Acc and Mag
        void    setFIFOMode(lsm9ds0_fifo_mode_t mode);
        lsm9ds0_fifo_mode_t getFIFOMode();
        void    setGyroFIFOThreshold(uint8_t wtm);
        uint8_t getGyroFIFOThreshold();
        void    setMagAccFIFOThreshold(uint8_t wtm);
        uint8_t getMagAccFIFOThreshold();

        // FIFO_SRC register, read-only
        bool getGyroFIFOAtWatermark();
        bool getGyroFIFOOverrun();
        bool getGyroFIFOEmpty();
        uint8_t getGyroFIFOStoredDataLevel();

        bool getMagAccFIFOAtWatermark();
        bool getMagAccFIFOOverrun();
        bool getMagAccFIFOEmpty();
        uint8_t getMagAccFIFOStoredDataLevel();

        // INT1_CFG register, r/w
        void setGyroInterruptCombination(bool combination);
        bool getGyroInterruptCombination();
        void setGyroInterruptRequestLatched(bool latched);
        bool getGyroInterruptRequestLatched();
        void setGyroZHighInterruptEnabled(bool enabled);
        bool getGyroZHighInterruptEnabled();
        void setGyroYHighInterruptEnabled(bool enabled);
        bool getGyroYHighInterruptEnabled();
        void setGyroXHighInterruptEnabled(bool enabled);
        bool getGyroXHighInterruptEnabled();
        void setGyroZLowInterruptEnabled(bool enabled);
        bool getGyroZLowInterruptEnabled();
        void setGyroYLowInterruptEnabled(bool enabled);
        bool getGyroYLowInterruptEnabled();
        void setGyroXLowInterruptEnabled(bool enabled);
        bool getGyroXLowInterruptEnabled();

        // INT1_SRC register, read-only
        bool getGyroInterruptActive();
        bool getGyroZHigh();
        bool getGyroZLow();
        bool getGyroYHigh();
        bool getGyroYLow();
        bool getGyroXHigh();
        bool getGyroXLow();

        // INT1_THS_* registers, r/w
        void    setGyroXHighThreshold(uint8_t threshold);
        uint8_t getGyroXHighThreshold();
        void    setGyroXLowThreshold(uint8_t threshold);
        uint8_t getGyroXLowThreshold();
        void    setGyroYHighThreshold(uint8_t threshold);
        uint8_t getGyroYHighThreshold();
        void    setGyroYLowThreshold(uint8_t threshold);
        uint8_t getGyroYLowThreshold();
        void    setGyroZHighThreshold(uint8_t threshold);
        uint8_t getGyroZHighThreshold();
        void    setGyroZLowThreshold(uint8_t threshold);
        uint8_t getGyroZLowThreshold();

        // INT1_DURATION register, r/w
        void    setGyroDuration(uint8_t duration);
        uint8_t getGyroDuration();
        void setGyroWaitEnabled(bool enabled);
        bool getGyroWaitEnabled();

        // OUT_TEMP_* register, read-only
        int16_t getTemperature();
        float   getTemperatureCelsius();

        // STATUS_REG_M register, read-only
        bool getMagXYZOverrun();
        bool getMagZOverrun();
        bool getMagYOverrun();
        bool getMagXOverrun();
        bool getMagXYZDataAvailable();
        bool getMagZDataAvailable();
        bool getMagYDataAvailable();
        bool getMagXDataAvailable();

        // OUT_MAG_* registers, read-only
        void    getMagneticFieldStrength(int16_t* x, int16_t* y, int16_t* z);
        int16_t getMagneticFieldStrengthX();
        int16_t getMagneticFieldStrengthY();
        int16_t getMagneticFieldStrengthZ();

        // INT_CTRL_REG_M, r/w
        void setMagXAxisInterrupt(bool enable);
        bool getMagXAxisInterrupt();
        void setMagYAxisInterrupt(bool enable);
        bool getMagYAxisInterrupt();
        bool getMagZAxisInterrupt();
        void setMagZAxisInterrupt(bool enable);
        bool getMagAccInterruptOpenDrain();
        void setMagAccInterruptOpenDrain(bool isopendrain);
        bool getInterruptPolarity();
        void getInterruptPolarity(bool enable);
        bool getMagAccInterruptLatch();
        void getMagAccInterruptLatch(bool enable);
        bool getAcc4DDetection();
        void setAcc4DDetection(bool enable);
        bool getMagInterruptEnable();
        void setMagInterruptEnable(bool enable);

        // INT_SRC_REG_M not implemented
        // INT_THS_M not implemented
        // OFFSET_*_M not implemented
        // REFERENCE_* not implemented
        // CTRL_REG0_XM r/w
        void setMagAccWatermarkEnable(bool enable);
        void setMagAccHighPassForClick(bool enable);
        void setMagAccHighPassIrq1(bool enable);
        void setMagAccHighPassIrq2(bool enable);

        // CTRL_REG1_XM, w, TODO: only rate settable
        void setAccRate(lsm9ds0_acc_data_rate_t rate);

        // CTRL_REG2_XM, w, TODO: only filter bw and scale
        void setAccAntiAliasFilterBandwidth(lsm9ds0_acc_filter_bw_t bw);
        void setAccFullScale(lsm9ds0_acc_full_scale_t scale);
        lsm9ds0_acc_full_scale_t getAccFullScale();

        // CTRL_REG_3_XM, not implemented
        // CTRL_REG_4_XM, not implemented
        // CTRL_REG_5_XM, TODO: only resolution and rate selection
        void setMagResolution(bool high);
        void setMagOutputRate(lsm9ds0_mag_output_rate_t rate);

        // CTRL_REG_6_XM
        void setMagFullScale(lsm9ds0_mag_scale_t scale);
        lsm9ds0_mag_scale_t getMagFullScale();

        // CTRL_REG_7_XM, TODO: partly
        void setAccHighPassFilterMode(lsm9ds0_acc_highpass_fitermode_t fm);
        void setFilterEnable(bool enable);

        // STATUS_REG_A register, read-only
        bool getAccXYZOverrun();
        bool getAccZOverrun();
        bool getAccYOverrun();
        bool getAccXOverrun();
        bool getAccXYZDataAvailable();
        bool getAccZDataAvailable();
        bool getAccYDataAvailable();
        bool getAccXDataAvailable();

        // OUT_ACC* registers, read-only
        void    getAccleration(int16_t* x, int16_t* y, int16_t* z);
        int16_t getAcclerationX();
        int16_t getAcclerationY();
        int16_t getAcclerationZ();

    private:
        uint8_t devAddrGyro,
                devAddrMagAcc;
        uint8_t buffer[6];

        // correction values from datasheet pg 13
        float acc_scale, mag_scale, gyr_scale;
};

#endif /* _LSM9DS0_H_ */
