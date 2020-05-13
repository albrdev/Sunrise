#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#ifndef __REGION__REGISTER_VALUES__
#define REG_ERRORSTATUS_MSB                     0x00
#define REG_ERRORSTATUS_LSB                     0x00

#define REG_RESERVED_01                         0x02
#define REG_RESERVED_02                         0x03
#define REG_RESERVED_03                         0x04
#define REG_RESERVED_04                         0x05

#define REG_CO2_FP_MSB                          0x06
#define REG_CO2_FP_LSB                          0x07

#define REG_TEMPERATURE_MSB                     0x08
#define REG_TEMPERATURE_LSB                     0x09

#define REG_RESERVED_05                         0x0A
#define REG_RESERVED_06                         0x0B
#define REG_RESERVED_07                         0x0C

#define REG_MEASUREMENT_COUNT                   0x0D
#define REG_MEASUREMENT_CYCLE_TIME_MSB          0x0E
#define REG_MEASUREMENT_CYCLE_TIME_LSB          0x0F

#define REG_CO2_UP_MSB                          0x10
#define REG_CO2_UP_LSB                          0x11
#define REG_CO2_F_MSB                           0x12
#define REG_CO2_F_LSB                           0x13
#define REG_CO2_U_MSB                           0x14
#define REG_CO2_U_LSB                           0x15

#define REG_FIRMWARE_REV_MSB                    0x38
#define REG_FIRMWARE_REV_LSB                    0x39

#define REG_SENSOR_ID_MMSB                      0x3A
#define REG_SENSOR_ID_MLSB                      0x3B
#define REG_SENSOR_ID_LMSB                      0x3C
#define REG_SENSOR_ID_LLSB                      0x3D

#define REG_RESERVED_08                         0x3E
#define REG_RESERVED_09                         0x3F
#define REG_RESERVED_10                         0x80

#define REG_CALIBRATION_STATUS                  0x81

#define REG_CALIBRATION_COMMAND_MSB             0x82
#define REG_CALIBRATION_COMMAND_LSB             0x83

#define REG_CALIBRATION_TARGET_MSB              0x84
#define REG_CALIBRATION_TARGET_LSB              0x85

#define REG_CO2_VALUE_OVERRIDE_MSB              0x86
#define REG_CO2_VALUE_OVERRIDE_LSB              0x87

#define REG_ABC_TIME_MSB                        0x88
#define REG_ABC_TIME_LSB                        0x89

#define REG_ABC_PAR0_MSB                        0x8A
#define REG_ABC_PAR0_LSB                        0x8B
#define REG_ABC_PAR1_MSB                        0x8C
#define REG_ABC_PAR1_LSB                        0x8D
#define REG_ABC_PAR2_MSB                        0x8E
#define REG_ABC_PAR2_LSB                        0x8F
#define REG_ABC_PAR3_MSB                        0x90
#define REG_ABC_PAR3_LSB                        0x91

#define REG_RESERVED_11                         0x92

#define REG_START_SINGLE_MEASUREMENT            0x93

#define REG_RESERVED_12                         0x94

#define REG_MEASUREMENT_MODE                    0x95   // EE

#define REG_MEASUREMENT_PERIOD_MSB              0x96   // EE
#define REG_MEASUREMENT_PERIOD_LSB              0x97   // EE

#define REG_NUMBER_OF_SAMPLES_MSB               0x98   // EE
#define REG_NUMBER_OF_SAMPLES_LSB               0x99   // EE

#define REG_ABC_PERIOD_MSB                      0x9A   // EE
#define REG_ABC_PERIOD_LSB                      0x9B   // EE

#define REG_RESERVED_13                         0x9C

#define REG_CLEAR_ERRORSTATUS                   0x9D

#define REG_ABC_TARGET_MSB                      0x9E   // EE
#define REG_ABC_TARGET_LSB                      0x9F   // EE

#define REG_RESERVED_14                         0xA0

#define REG_STATIC_IIR_FILTER_PARAMETER         0xA1   // EE

#define REG_RESERVED_15                         0xA2

#define REG_SCR                                 0xA3

#define REG_RESERVED_16                         0xA4

#define REG_METER_CONTROL                       0xA5   // EE

#define REG_RESERVED_17                         0xA6

#define REG_MB_I2C_ADDRESS                      0xA7   // EE

// Registers from address 0xC0 to 0xCD are mirrors of registers at addresses 0x80, 0x81, 0x92, 0x93, and 0x88 to 0x91.

#define REGM_RESERVED_10                        0xC0 // 0x80
#define REGM_CALIBRATION_STATUS                 0xC1 // 0x81

#define REGM_RESERVED_11                        0xC2 // 0x92
#define REGM_START_SINGLE_MEASUREMENT           0xC3 // 0x93

#define REGM_ABC_TIME_MSB                       0xC4 // 0x88
#define REGM_ABC_TIME_LSB                       0xC5 // 0x89

#define REGM_ABC_PAR0_MSB                       0xC6 // 0x8A
#define REGM_ABC_PAR0_LSB                       0xC7 // 0x8B
#define REGM_ABC_PAR1_MSB                       0xC8 // 0x8C
#define REGM_ABC_PAR1_LSB                       0xC9 // 0x8D
#define REGM_ABC_PAR2_MSB                       0xCA // 0x8E
#define REGM_ABC_PAR2_LSB                       0xCB // 0x8F
#define REGM_ABC_PAR3_MSB                       0xCC // 0x90
#define REGM_ABC_PAR3_LSB                       0xCD // 0x91

#define REG_FILTER_PAR0_MSB                     0xCE
#define REG_FILTER_PAR0_LSB                     0xCF
#define REG_FILTER_PAR1_MSB                     0xD0
#define REG_FILTER_PAR1_LSB                     0xD1
#define REG_FILTER_PAR2_MSB                     0xD2
#define REG_FILTER_PAR2_LSB                     0xD3
#define REG_FILTER_PAR3_MSB                     0xD4
#define REG_FILTER_PAR3_LSB                     0xD5
#define REG_FILTER_PAR4_MSB                     0xD6
#define REG_FILTER_PAR4_LSB                     0xD7
#define REG_FILTER_PAR5_MSB                     0xD8
#define REG_FILTER_PAR5_LSB                     0xD9
#define REG_FILTER_PAR6_MSB                     0xDA
#define REG_FILTER_PAR6_LSB                     0xDB

#define REG_BAROMETRIC_AIR_PRESSURE_VALUE_MSB   0xDC
#define REG_BAROMETRIC_AIR_PRESSURE_VALUE_LSB   0xDD

#define REG_RESERVED_18                         0xDE
#define REG_RESERVED_19                         0xDF
#endif // __REGION__REGISTER_VALUES__

#define MEASUREMENT_PERIOD_MIN      2
#define MEASUREMENT_PERIOD_MAX      65534
#define NUMBER_OF_SAMPLES_MIN       1
#define NUMBER_OF_SAMPLES_MAX       1024
#define ABC_PERIOD_MIN              1
#define ABC_PERIOD_MAX              65534
#define BAROMETRIC_AIR_PRESSURE_MIN 3000
#define BAROMETRIC_AIR_PRESSURE_MAX 13000

#define DELAY_WAKEUP                35UL
#define DELAY_TIMEOUT               15UL
#define DELAY_SRAM                  1UL
#define DELAY_EEPROM                25UL

#define I2C_CLOCK_FREQUENCY         100000UL
#define I2C_ADDRESS_MIN             0x08        // 0-7 are reserved
#define I2C_ADDRESS_MAX             0x78        // 120-127 are reserved. 127-255 are out of 7-bit address range.

#endif  // __CONSTANTS_H__
