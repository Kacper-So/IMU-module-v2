/* 07/13/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
  The ICM40627 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef ICM40627_h
#define ICM40627_h

#include "Arduino.h"
#include <Wire.h>
/* ICM40627 registers
https://store.invensense.com/datasheets/invensense/DS-ICM-42605v1-2.pdf
*/
// Bank 0
#define ICM40627_DEVICE_CONFIG             0x11
#define ICM40627_DRIVE_CONFIG              0x13
#define ICM40627_INT_CONFIG                0x14
#define ICM40627_FIFO_CONFIG               0x16
#define ICM40627_TEMP_DATA1                0x1D
#define ICM40627_TEMP_DATA0                0x1E

#define ICM40627_ACCEL_DATA_X1             0x1F
#define ICM40627_ACCEL_DATA_X0             0x20
#define ICM40627_ACCEL_DATA_Y1             0x21
#define ICM40627_ACCEL_DATA_Y0             0x22
#define ICM40627_ACCEL_DATA_Z1             0x23
#define ICM40627_ACCEL_DATA_Z0             0x24

#define ICM40627_GYRO_DATA_X1              0x25
#define ICM40627_GYRO_DATA_X0              0x26
#define ICM40627_GYRO_DATA_Y1              0x27
#define ICM40627_GYRO_DATA_Y0              0x28
#define ICM40627_GYRO_DATA_Z1              0x29
#define ICM40627_GYRO_DATA_Z0              0x2A

#define ICM40627_TMST_FSYNCH               0x2B
#define ICM40627_TMST_FSYNCL               0x2C
#define ICM40627_INT_STATUS                0x2D
#define ICM40627_FIFO_COUNTH               0x2E
#define ICM40627_FIFO_COUNTL               0x2F
#define ICM40627_FIFO_DATA                 0x30
#define ICM40627_APEX_DATA0                0x31
#define ICM40627_APEX_DATA1                0x32
#define ICM40627_APEX_DATA2                0x33
#define ICM40627_APEX_DATA3                0x34
#define ICM40627_APEX_DATA4                0x35
#define ICM40627_APEX_DATA5                0x36
#define ICM40627_INT_STATUS2               0x37
#define ICM40627_INT_STATUS3               0x38

#define ICM40627_SIGNAL_PATH_RESET         0x4B
#define ICM40627_INTF_CONFIG0              0x4C
#define ICM40627_INTF_CONFIG1              0x4D
#define ICM40627_PWR_MGMT0                 0x4E
#define ICM40627_GYRO_CONFIG0              0x4F
#define ICM40627_ACCEL_CONFIG0             0x50
#define ICM40627_GYRO_CONFIG1              0x51
#define ICM40627_GYRO_ACCEL_CONFIG0        0x52
#define ICM40627_ACCEL_CONFIG1             0x53
#define ICM40627_TMST_CONFIG               0x54
#define ICM40627_APEX_CONFIG0              0x56
#define ICM40627_SMD_CONFIG                0x57
#define ICM40627_FIFO_CONFIG1              0x5F
#define ICM40627_FIFO_CONFIG2              0x60
#define ICM40627_FIFO_CONFIG3              0x61
#define ICM40627_FSYNC_CONFIG              0x62
#define ICM40627_INT_CONFIG0               0x63
#define ICM40627_INT_CONFIG1               0x64
#define ICM40627_INT_SOURCE0               0x65
#define ICM40627_INT_SOURCE1               0x66
#define ICM40627_INT_SOURCE3               0x68
#define ICM40627_INT_SOURCE4               0x69
#define ICM40627_FIFO_LOST_PKT0            0x6C
#define ICM40627_FIFO_LOST_PKT1            0x6D
#define ICM40627_SELF_TEST_CONFIG          0x70
#define ICM40627_WHO_AM_I                  0x75
#define ICM40627_REG_BANK_SEL              0x76

// Bank 1
#define ICM40627_SENSOR_CONFIG0            0x03
#define ICM40627_GYRO_CONFIG_STATIC2       0x0B
#define ICM40627_GYRO_CONFIG_STATIC3       0x0C
#define ICM40627_GYRO_CONFIG_STATIC4       0x0D
#define ICM40627_GYRO_CONFIG_STATIC5       0x0E
#define ICM40627_GYRO_CONFIG_STATIC6       0x0F
#define ICM40627_GYRO_CONFIG_STATIC7       0x10
#define ICM40627_GYRO_CONFIG_STATIC8       0x11
#define ICM40627_GYRO_CONFIG_STATIC9       0x12
#define ICM40627_GYRO_CONFIG_STATIC10      0x13
#define ICM40627_XG_ST_DATA                0x5F
#define ICM40627_YG_ST_DATA                0x60
#define ICM40627_ZG_ST_DATA                0x61
#define ICM40627_TMSTVAL0                  0x62
#define ICM40627_TMSTVAL1                  0x63
#define ICM40627_TMSTVAL2                  0x64
#define ICM40627_INTF_CONFIG4              0x7A
#define ICM40627_INTF_CONFIG5              0x7B
// #define ICM40627_INTF_CONFIG6              0x7C

// Bank 2
#define ICM40627_ACCEL_CONFIG_STATIC2      0x03
#define ICM40627_ACCEL_CONFIG_STATIC3      0x04
#define ICM40627_ACCEL_CONFIG_STATIC4      0x05
#define ICM40627_XA_ST_DATA                0x3B
#define ICM40627_YA_ST_DATA                0x3C
#define ICM40627_ZA_ST_DATA                0x3D

// Bank 4
#define ICM40627_GYRO_ON_OFF_CONFIG        0x0E
#define ICM40627_APEX_CONFIG1              0x40
#define ICM40627_APEX_CONFIG2              0x41
#define ICM40627_APEX_CONFIG3              0x42
#define ICM40627_APEX_CONFIG4              0x43
#define ICM40627_APEX_CONFIG5              0x44
#define ICM40627_APEX_CONFIG6              0x45
#define ICM40627_APEX_CONFIG7              0x46
#define ICM40627_APEX_CONFIG8              0x47
#define ICM40627_APEX_CONFIG9              0x48
#define ICM40627_ACCEL_WOM_X_THR           0x4A
#define ICM40627_ACCEL_WOM_Y_THR           0x4B
#define ICM40627_ACCEL_WOM_Z_THR           0x4C
#define ICM40627_INT_SOURCE6               0x4D
#define ICM40627_INT_SOURCE7               0x4E
// #define ICM40627_INT_SOURCE8               0x4F
// #define ICM40627_INT_SOURCE9               0x50
// #define ICM40627_INT_SOURCE10              0x51
#define ICM40627_OFFSET_USER0              0x77
#define ICM40627_OFFSET_USER1              0x78
#define ICM40627_OFFSET_USER2              0x79
#define ICM40627_OFFSET_USER3              0x7A
#define ICM40627_OFFSET_USER4              0x7B
#define ICM40627_OFFSET_USER5              0x7C
#define ICM40627_OFFSET_USER6              0x7D
#define ICM40627_OFFSET_USER7              0x7E
#define ICM40627_OFFSET_USER8              0x7F

#define ICM40627_ADDRESS                   0x69   // Address of ICM40627 accel/gyro when ADO = HIGH


#define AFS_2G  0x03
#define AFS_4G  0x02
#define AFS_8G  0x01
#define AFS_16G 0x00  // default

#define GFS_2000DPS   0x00 // default
#define GFS_1000DPS   0x01
#define GFS_500DPS    0x02
#define GFS_250DPS    0x03
#define GFS_125DPS    0x04
#define GFS_62_5DPS   0x05
#define GFS_31_25DPS  0x06
#define GFS_15_125DPS 0x07

#define AODR_8000Hz   0x03
#define AODR_4000Hz   0x04
#define AODR_2000Hz   0x05
#define AODR_1000Hz   0x06 // default
#define AODR_200Hz    0x07
#define AODR_100Hz    0x08
#define AODR_50Hz     0x09
#define AODR_25Hz     0x0A
#define AODR_12_5Hz   0x0B
#define AODR_6_25Hz   0x0C
#define AODR_3_125Hz  0x0D
#define AODR_1_5625Hz 0x0E
#define AODR_500Hz    0x0F

#define GODR_8000Hz  0x03
#define GODR_4000Hz  0x04
#define GODR_2000Hz  0x05
#define GODR_1000Hz  0x06 // default
#define GODR_200Hz   0x07
#define GODR_100Hz   0x08
#define GODR_50Hz    0x09
#define GODR_25Hz    0x0A
#define GODR_12_5Hz  0x0B
#define GODR_500Hz   0x0F


//* AIR MOTION LIBRARY 
#define BIT_GYRO_CONFIG0_FS_SEL_POS       5
#define BIT_GYRO_CONFIG0_FS_SEL_MASK     (7 << BIT_GYRO_CONFIG0_FS_SEL_POS)
#define BIT_GYRO_CONFIG0_ODR_POS       0
#define BIT_GYRO_CONFIG0_ODR_MASK   0x0F
/* GYRO_ODR */
#define BIT_GYRO_CONFIG0_ODR_POS       0
#define BIT_GYRO_CONFIG0_ODR_MASK   0x0F
/* GYRO_UI_FILT_ORD */
#define BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS       2
#define BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK   (0x3 << BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS)
/* GYRO_UI_FILT_BW_IND */
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_POS       0
#define BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK   0x0F
/* GYRO_MODE */
#define BIT_PWR_MGMT_0_GYRO_MODE_POS        2
#define BIT_PWR_MGMT_0_GYRO_MODE_MASK   (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS)
/* ACCEL_FS_SEL */
#define BIT_ACCEL_CONFIG0_FS_SEL_POS       5
#define BIT_ACCEL_CONFIG0_FS_SEL_MASK   (0x7 << BIT_ACCEL_CONFIG0_FS_SEL_POS)
  
#define BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS       3
#define BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK    (0x3 << BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS)
/* ACCEL_MODE */
#define BIT_PWR_MGMT_0_ACCEL_MODE_POS       0
#define BIT_PWR_MGMT_0_ACCEL_MODE_MASK   0x03
/* ACCEL_ODR */
#define BIT_ACCEL_CONFIG0_ODR_POS       0
#define BIT_ACCEL_CONFIG0_ODR_MASK   0x0F
/* ACCEL_UI_FILT_BW_IND */
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS       4
#define BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK   (0xF << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS) 

  typedef enum
{
ICM406XX_GYRO_CONFIG0_FS_SEL_16dps   = (7 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 16dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_31dps   = (6 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 31dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_62dps   = (5 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 62dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_125dps  = (4 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 125dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_250dps  = (3 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 250dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_500dps  = (2 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 500dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_1000dps = (1 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 1000dps*/
ICM406XX_GYRO_CONFIG0_FS_SEL_2000dps = (0 << BIT_GYRO_CONFIG0_FS_SEL_POS),  /*!< 2000dps*/
} ICM406XX_GYRO_CONFIG0_FS_SEL_t;

typedef enum
{
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_LN  = 0x03,
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_LP  = 0x02,
	ICM406XX_PWR_MGMT_0_ACCEL_MODE_OFF = 0x00,
} ICM406XX_PWR_MGMT_0_ACCEL_MODE_t;

/** @brief Gyroscope ODR selection 
 */
typedef enum
{
ICM406XX_GYRO_CONFIG0_ODR_500_HZ    = 0x0F,  /*!< 500 Hz (2 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_12_5_HZ   = 0x0B,  /*!< 12.5 Hz (80 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_25_HZ     = 0x0A,  /*!< 25 Hz (40 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_50_HZ     = 0x09,  /*!< 50 Hz (20 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_100_HZ    = 0x08,  /*!< 100 Hz (10 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_200_HZ    = 0x07,  /*!< 200 Hz (5 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_1_KHZ     = 0x06,  /*!< 1 KHz (1 ms)*/
ICM406XX_GYRO_CONFIG0_ODR_2_KHZ     = 0x05,  /*!< 2 KHz (500 us)*/
ICM406XX_GYRO_CONFIG0_ODR_4_KHZ     = 0x04,  /*!< 4 KHz (250 us)*/
ICM406XX_GYRO_CONFIG0_ODR_8_KHZ     = 0x03,  /*!< 8 KHz (125 us)*/
ICM406XX_GYRO_CONFIG0_ODR_16_KHZ    = 0x02,  /*!< 16 KHz (62.5 us)*/
ICM406XX_GYRO_CONFIG0_ODR_32_KHZ    = 0x01,  /*!< 32 KHz (31.25 us)*/
} ICM406XX_GYRO_CONFIG0_ODR_t;

typedef enum
{
ICM406XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_1ST_ORDER = (0x0 << BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS),
ICM406XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_2ND_ORDER = (0x1 << BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS),
ICM406XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER = (0x2 << BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_POS),
} ICM406XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_t;

typedef enum
{
	ICM406XX_PWR_MGMT_0_GYRO_MODE_LN      = (0x03 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
	ICM406XX_PWR_MGMT_0_GYRO_MODE_STANDBY = (0x01 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
	ICM406XX_PWR_MGMT_0_GYRO_MODE_OFF     = (0x00 << BIT_PWR_MGMT_0_GYRO_MODE_POS),
} ICM406XX_PWR_MGMT_0_GYRO_MODE_t;

typedef enum
{
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_40 = 0x07,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_20 = 0x06,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_16 = 0x05,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10 = 0x04,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_8  = 0x03,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_5  = 0x02,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4  = 0x01,
ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_2  = 0x00,
} ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t;



enum inv_icm406xx_sensor {
INV_ICM406XX_SENSOR_ACCEL,               /**< Accelerometer (UI control path) */
INV_ICM406XX_SENSOR_GYRO,                /**< Gyroscope (UI control path) */
INV_ICM406XX_SENSOR_FSYNC_EVENT,         /**< Used by OIS and UI control layers */
INV_ICM406XX_SENSOR_OIS,                 /**< Only used by OIS layer */
INV_ICM406XX_SENSOR_TEMPERATURE,         /**< Chip temperature, enabled by default. However, it will be reported only if Accel and/or Gyro are also enabled. 
                                            The Temperature's ODR (Output Data Rate) will match the ODR of Accel or Gyro, or the fastest if both are enabled*/
INV_ICM406XX_SENSOR_TAP,                 /**< Tap and Double tap */
INV_ICM406XX_SENSOR_DMP_PEDOMETER_EVENT, /**< Pedometer: step is detected */
INV_ICM406XX_SENSOR_DMP_PEDOMETER_COUNT, /**< Pedometer: step counter */
INV_ICM406XX_SENSOR_DMP_TILT,            /**< Tilt */
INV_ICM406XX_SENSOR_DMP_R2W,             /**< Raise to wake */
INV_ICM406XX_SENSOR_MAX
};

enum inv_icm406xx_sensor_on_mask {
INV_ICM406XX_SENSOR_ON_MASK_ACCEL = (1L<<INV_ICM406XX_SENSOR_ACCEL),
INV_ICM406XX_SENSOR_ON_MASK_GYRO  = (1L<<INV_ICM406XX_SENSOR_GYRO),
};

typedef enum
{
ICM406XX_ACCEL_CONFIG0_FS_SEL_RESERVED = (0x4 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  
ICM406XX_ACCEL_CONFIG0_FS_SEL_2g       = (0x3 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 2g*/
ICM406XX_ACCEL_CONFIG0_FS_SEL_4g       = (0x2 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 4g*/
ICM406XX_ACCEL_CONFIG0_FS_SEL_8g       = (0x1 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 8g*/
ICM406XX_ACCEL_CONFIG0_FS_SEL_16g      = (0x0 << BIT_ACCEL_CONFIG0_FS_SEL_POS),  /*!< 16g*/
} ICM406XX_ACCEL_CONFIG0_FS_SEL_t;

typedef enum
{
ICM406XX_ACCEL_CONFIG0_ODR_500_HZ    = 0xF,  /*!< 500 Hz (2 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_1_5625_HZ = 0xE,  /*!< 1.5625 Hz (640 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_3_125_HZ  = 0xD,  /*!< 3.125 Hz (320 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_6_25_HZ   = 0xC,  /*!< 6.25 Hz (160 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_12_5_HZ   = 0xB,  /*!< 12.5 Hz (80 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_25_HZ     = 0xA,  /*!< 25 Hz (40 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_50_HZ     = 0x9,  /*!< 50 Hz (20 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_100_HZ    = 0x8,  /*!< 100 Hz (10 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_200_HZ    = 0x7,  /*!< 200 Hz (5 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_1_KHZ     = 0x6,  /*!< 1 KHz (1 ms)*/
ICM406XX_ACCEL_CONFIG0_ODR_2_KHZ     = 0x5,  /*!< 2 KHz (500 us)*/
ICM406XX_ACCEL_CONFIG0_ODR_4_KHZ     = 0x4,  /*!< 4 KHz (250 us)*/
ICM406XX_ACCEL_CONFIG0_ODR_8_KHZ     = 0x3,  /*!< 8 KHz (125 us)*/
ICM406XX_ACCEL_CONFIG0_ODR_16_KHZ    = 0x2,  /*!< 16 KHz (62.5 us)*/
ICM406XX_ACCEL_CONFIG0_ODR_32_KHZ    = 0x1,  /*!< 32 KHz (31.25 us)*/
} ICM406XX_ACCEL_CONFIG0_ODR_t;

typedef enum
{
ICM406XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_1ST_ORDER = (0x0 << BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS),
ICM406XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_2ND_ORDER = (0x1 << BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS),
ICM406XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER = (0x2 << BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_POS),
} ICM406XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_t;


typedef enum
{
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_40 = (0x7 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_20 = (0x6 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_16 = (0x5 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10 = (0x4 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_8  = (0x3 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_5  = (0x2 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4  = (0x1 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
	ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_2  = (0x0 << BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_POS),
} ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t;




/** register configuration for self-test procedure */
#define ST_GYRO_FSR             ICM406XX_GYRO_CONFIG0_FS_SEL_250dps
#define ST_GYRO_ODR             ICM406XX_GYRO_CONFIG0_ODR_1_KHZ
#define ST_GYRO_UI_FILT_ORD_IND ICM406XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_3RD_ORDER
#define ST_GYRO_UI_FILT_BW_IND  ICM406XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_10

#define ST_ACCEL_FSR             ICM406XX_ACCEL_CONFIG0_FS_SEL_4g
#define ST_ACCEL_ODR             ICM406XX_ACCEL_CONFIG0_ODR_1_KHZ
#define ST_ACCEL_UI_FILT_ORD_IND ICM406XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_3RD_ORDER
#define ST_ACCEL_UI_FILT_BW_IND  ICM406XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_10

#define BIT_ST_REGULATOR_EN                 0x40
#define BIT_ACCEL_Z_ST_EN                   0x20
#define BIT_ACCEL_Y_ST_EN                   0x10
#define BIT_ACCEL_X_ST_EN                   0x08
#define BIT_GYRO_Z_ST_EN                    0x04
#define BIT_GYRO_Y_ST_EN                    0x02
#define BIT_GYRO_X_ST_EN                    0x01

#define INV_ST_OTP_EQUATION(FS, ST_code) (uint32_t)((2620/pow(2,3-FS))*pow(1.01, ST_code-1)+0.5)


/* Pass/Fail criteria */
#define MIN_RATIO_GYRO         0.5f /* expected ratio greater than 0.5 */ 
#define MIN_ST_GYRO_DPS        60   /* expected values greater than 60dps */
#define MAX_ST_GYRO_OFFSET_DPS 20   /* expected offset less than 20 dps */

#define MIN_RATIO_GYRO  0.5f /* expected ratio greater than 0.5 */ 
#define MAX_RATIO_GYRO  1.5f /* expected ratio lower than 1.5 */ 
#define MIN_ST_ACCEL_MG 225  /* expected values in [225mgee;675mgee] */
#define MAX_ST_ACCEL_MG 675

#define BIT_INT_STATUS_UI_FSYNC   0x40
#define BIT_INT_STATUS_PLL_RDY    0x20
#define BIT_INT_STATUS_RESET_DONE 0x10
#define BIT_INT_STATUS_DRDY       0x08
#define BIT_INT_STATUS_FIFO_THS   0x04
#define BIT_INT_STATUS_FIFO_FULL  0x02
#define BIT_INT_STATUS_AGC_RDY    0x01

enum inv_error
{
	INV_ERROR_SUCCESS      = 0,   /**< no error */
	INV_ERROR              = -1,  /**< unspecified error */
	INV_ERROR_NIMPL        = -2,  /**< function not implemented for given
	                                   arguments */
	INV_ERROR_TRANSPORT    = -3,  /**< error occured at transport level */
	INV_ERROR_TIMEOUT      = -4,  /**< action did not complete in the expected
	                                   time window */
	INV_ERROR_SIZE         = -5,  /**< size/length of given arguments is not
	                                   suitable to complete requested action */
	INV_ERROR_OS           = -6,  /**< error related to OS */
	INV_ERROR_IO           = -7,  /**< error related to IO operation */
	INV_ERROR_MEM          = -9,  /**< not enough memory to complete requested
	                                   action */
	INV_ERROR_HW           = -10, /**< error at HW level */
	INV_ERROR_BAD_ARG      = -11, /**< provided arguments are not good to
	                                   perform requestion action */
	INV_ERROR_UNEXPECTED   = -12, /**< something unexpected happened */
	INV_ERROR_FILE         = -13, /**< cannot access file or unexpected format */
	INV_ERROR_PATH         = -14, /**< invalid file path */
	INV_ERROR_IMAGE_TYPE   = -15, /**< error when image type is not managed */
	INV_ERROR_WATCHDOG     = -16, /**< error when device doesn't respond 
									   to ping */
};

/* DATA_ENDIAN */
#define BIT_DATA_ENDIAN_POS                 4
#define BIT_DATA_ENDIAN_MASK            (0x01 << BIT_DATA_ENDIAN_POS)

typedef enum
{
	ICM406XX_INTF_CONFIG0_DATA_BIG_ENDIAN    = (0x01 << BIT_DATA_ENDIAN_POS),
	ICM406XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN = (0x00 << BIT_DATA_ENDIAN_POS),
} ICM406XX_INTF_CONFIG0_DATA_ENDIAN_t;


typedef enum{
	PASSED = 0,  
  	FAILED
}ICM40627_SELF_TEST_RESULT;
class ICM40627
{
  public:
  ICM40627();
  float getAres(uint8_t Ascale);
  float getGres(uint8_t Gscale);
  uint8_t getChipID();
  bool whoamiCheck();
  void init(uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
  void reset();
  void readData(int16_t * destination);
  uint8_t status();
  void initR2SW(void);
  void setWOM();
  void getRegistersValuesForOffsets(unsigned char *reg1, unsigned char *reg2, unsigned char *reg3,
                        float val1, float val2, float resolution1, float resolution2);
  void setOffsets(float ax, float ay, float az, float gx, float gy, float gz);
  void initPedometer();
  void initTapDetection();
  void sleepSensor();
  void wakeUpSensor();
  int readWakeSleepStatus();
  void readPedometerStatus(uint16_t *result);
  uint8_t readByte(uint8_t address, uint8_t subAddress);
  void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
  void writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
  bool selfTest();    
  bool runAccelSelfTest();
  bool runGyroSelfTest();
  int reg_to_accel_fsr(ICM406XX_ACCEL_CONFIG0_FS_SEL_t reg);
  int reg_to_gyro_fsr(ICM406XX_GYRO_CONFIG0_FS_SEL_t reg);
  int average_sensor_output(int sensor, int self_test_config, int32_t average[3]);
  
  private:
    float _aRes, _gRes;
	float resolutionG = 1./32.; // dps
    float resolutionA = 0.0005; // mg
};

#endif
