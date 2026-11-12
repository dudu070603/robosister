#include "bsp_imu.h"
#include "BMI088reg.h"
#include "bsp_dwt.h"
#include "spi.h"
#include <cstring>

namespace engineer {
#if defined(HAL_SPI_MODULE_ENABLED)
namespace {
constexpr uint32_t kSpiTimeoutMs = 1;
struct Reg {
    bool accel;
    uint8_t reg, value;
};
constexpr Reg configuration[] = {
    {true, BMI088_ACC_PWR_CTRL, 0x04},    {true, BMI088_ACC_PWR_CONF, 0x00},
    {true, BMI088_ACC_CONF, 0xaa},        // normal bandwidth, 400 Hz
    {true, BMI088_ACC_RANGE, 0x01},       // +/-6 g
    {false, BMI088_GYRO_RANGE, 0x00},     // +/-2000 deg/s
    {false, BMI088_GYRO_BANDWIDTH, 0x83}, // template bandwidth format, 400 Hz / 47 Hz
    {false, BMI088_GYRO_LPM1, 0x00},      {false, BMI088_GYRO_CTRL, 0x80}};
void select(bool accel, bool active) {
    HAL_GPIO_WritePin(accel ? GPIOA : GPIOB, accel ? GPIO_PIN_4 : GPIO_PIN_0,
                      active ? GPIO_PIN_RESET : GPIO_PIN_SET);
}
bool read(bool accel, uint8_t reg, uint8_t *out, uint8_t len) {
    if (!out || !len || len > 9)
        return false;
    uint8_t tx[11]{}, rx[11]{};
    tx[0] = reg | 0x80;
    const uint8_t offset = accel ? 2 : 1; // accelerometer SPI has one extra dummy byte
    select(accel, true);
    const bool ok = HAL_SPI_TransmitReceive(&hspi1, tx, rx, len + offset, kSpiTimeoutMs) == HAL_OK;
    select(accel, false);
    if (ok)
        std::memcpy(out, rx + offset, len);
    return ok;
}
bool write(bool accel, uint8_t reg, uint8_t value) {
    uint8_t tx[2] = {reg, value}, rx[2]{};
    select(accel, true);
    const bool ok = HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, kSpiTimeoutMs) == HAL_OK;
    select(accel, false);
    return ok;
}
int16_t le16(const uint8_t *d) {
    return int16_t(uint16_t(d[0]) | uint16_t(d[1]) << 8);
}
} // namespace
void Bmi088::begin(uint32_t now) {
    status_ = {};
    config_index_ = 0;
    have_sensor_time_ = false;
    frames_ = 0;
    status_.phase = Bmi088Phase::RetryWait;
    due_ms_ = now;
}
void Bmi088::fail(uint32_t now, uint32_t error) {
    status_.errors |= error;
    have_sensor_time_ = false;
    status_.phase = status_.attempts >= 3 ? Bmi088Phase::Failed : Bmi088Phase::RetryWait;
    due_ms_ = now + 500;
}
bool Bmi088::poll(uint32_t now, Bmi088Sample &out) {
    if (status_.phase == Bmi088Phase::Off || status_.phase == Bmi088Phase::Failed)
        return false;
    if (int32_t(now - due_ms_) < 0)
        return false;
    if (status_.phase == Bmi088Phase::RetryWait) {
        ++status_.attempts;
        config_index_ = 0;
        uint8_t dummy;
        // The first ACC read selects SPI mode; do not treat that first value as ID.
        if (!read(true, BMI088_ACC_CHIP_ID, &dummy, 1) ||
            !write(true, BMI088_ACC_RESET, BMI088_RESET_VALUE) ||
            !write(false, BMI088_GYRO_RESET, BMI088_RESET_VALUE)) {
            fail(now, BmiTransport);
            return false;
        }
        status_.phase = Bmi088Phase::ResetWait;
        due_ms_ = now + 80;
        return false;
    }
    if (status_.phase == Bmi088Phase::ResetWait) {
        uint8_t dummy;
        if (!read(true, BMI088_ACC_CHIP_ID, &dummy, 1) ||
            !read(true, BMI088_ACC_CHIP_ID, &status_.accel_id, 1) ||
            !read(false, BMI088_GYRO_CHIP_ID, &status_.gyro_id, 1)) {
            fail(now, BmiTransport);
            return false;
        }
        if (status_.accel_id != BMI088_ACC_ID || status_.gyro_id != BMI088_GYRO_ID) {
            fail(now, BmiIdentity);
            return false;
        }
        status_.phase = Bmi088Phase::Configure;
        due_ms_ = now + 2;
        return false;
    }
    if (status_.phase == Bmi088Phase::Configure) {
        const auto &r = configuration[config_index_];
        if (!write(r.accel, r.reg, r.value)) {
            fail(now, BmiTransport);
            return false;
        }
        status_.phase = Bmi088Phase::Verify;
        due_ms_ = now + 2;
        return false;
    }
    if (status_.phase == Bmi088Phase::Verify) {
        const auto &r = configuration[config_index_];
        uint8_t actual = 0;
        if (!read(r.accel, r.reg, &actual, 1)) {
            fail(now, BmiTransport);
            return false;
        }
        if (actual != r.value) {
            fail(now, BmiConfiguration);
            return false;
        }
        if (++config_index_ == sizeof(configuration) / sizeof(configuration[0])) {
            status_.phase = Bmi088Phase::Ready;
            due_ms_ = now + 5;
        } else {
            status_.phase = Bmi088Phase::Configure;
            due_ms_ = now + 2;
        }
        return false;
    }
    uint8_t acc_state = 0, gyro_state = 0, acc_error = 0;
    if (!read(true, BMI088_ACC_STATUS, &acc_state, 1) ||
        !read(false, BMI088_GYRO_STATUS, &gyro_state, 1) ||
        !read(true, BMI088_ACC_ERROR, &acc_error, 1)) {
        ++status_.read_failures;
        fail(now, BmiTransport);
        return false;
    }
    if (acc_error & 0x05) {
        ++status_.read_failures;
        fail(now, BmiData);
        return false;
    }
    if (!(acc_state & 0x80) || !(gyro_state & 0x80))
        return false; // No new pair: do not refresh timestamp.
    uint8_t a[9]{}, g[8]{}, temp[2]{};
    if (!read(true, BMI088_ACC_DATA, a, 9) || !read(false, BMI088_GYRO_CHIP_ID, g, 8) ||
        !read(true, BMI088_ACC_TEMPERATURE, temp, 2)) {
        ++status_.read_failures;
        fail(now, BmiTransport);
        return false;
    }
    if (g[0] != BMI088_GYRO_ID) {
        ++status_.read_failures;
        fail(now, BmiIdentity);
        return false;
    }
    const uint32_t sensor_time = uint32_t(a[6]) | uint32_t(a[7]) << 8 | uint32_t(a[8]) << 16;
    if (have_sensor_time_ && sensor_time == sensor_time_)
        return false;
    sensor_time_ = sensor_time;
    have_sensor_time_ = true;
    Bmi088Sample next{};
    for (unsigned i = 0; i < 3; ++i) {
        next.accel_m_s2[i] = le16(a + 2 * i) * (6.0f * 9.80665f / 32768.0f);
        next.gyro_rad_s[i] = le16(g + 2 + 2 * i) * (2000.0f * 0.017453292519943295f / 32768.0f);
    }
    int16_t raw_temp = int16_t(uint16_t(temp[0]) << 3 | temp[1] >> 5);
    if (raw_temp > 1023)
        raw_temp -= 2048;
    next.temperature_c = 23 + raw_temp * .125f;
    next.received_ms = HAL_GetTick();
    next.received_cycles = BSP_DWT_Cycles();
    next.sensor_time = sensor_time;
    next.frames = ++frames_;
    status_.last_sample_ms = next.received_ms;
    out = next;
    return true;
}
#else
void Bmi088::begin(uint32_t) {
    status_.phase = Bmi088Phase::Failed;
    status_.errors = BmiUnavailable;
}
void Bmi088::fail(uint32_t, uint32_t error) {
    status_.phase = Bmi088Phase::Failed;
    status_.errors |= error;
}
bool Bmi088::poll(uint32_t, Bmi088Sample &) {
    return false;
}
#endif
} // namespace engineer
