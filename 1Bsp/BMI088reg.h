#pragma once
#include <stdint.h>

// BMI088 register subset used by the bounded polling driver. Values and SPI
// dummy-byte handling follow the original C-board template, not its retry loops.
enum : uint8_t {
    BMI088_ACC_CHIP_ID = 0x00,
    BMI088_ACC_ID = 0x1e,
    BMI088_ACC_ERROR = 0x02,
    BMI088_ACC_STATUS = 0x03,
    BMI088_ACC_DATA = 0x12,
    BMI088_ACC_TEMPERATURE = 0x22,
    BMI088_ACC_CONF = 0x40,
    BMI088_ACC_RANGE = 0x41,
    BMI088_ACC_PWR_CONF = 0x7c,
    BMI088_ACC_PWR_CTRL = 0x7d,
    BMI088_ACC_RESET = 0x7e,
    BMI088_GYRO_CHIP_ID = 0x00,
    BMI088_GYRO_ID = 0x0f,
    BMI088_GYRO_STATUS = 0x0a,
    BMI088_GYRO_RANGE = 0x0f,
    BMI088_GYRO_BANDWIDTH = 0x10,
    BMI088_GYRO_LPM1 = 0x11,
    BMI088_GYRO_RESET = 0x14,
    BMI088_GYRO_CTRL = 0x15,
    BMI088_RESET_VALUE = 0xb6
};
