#pragma once
#include <stdint.h>
namespace engineer {
struct Bmi088Sample {
    float accel_m_s2[3]{}, gyro_rad_s[3]{}, temperature_c = 0;
    uint32_t received_ms = 0, received_cycles = 0, sensor_time = 0, frames = 0;
};
enum class Bmi088Phase : uint8_t { Off, ResetWait, Configure, Verify, Ready, RetryWait, Failed };
enum Bmi088Error : uint32_t {
    BmiNoError = 0,
    BmiTransport = 1,
    BmiIdentity = 2,
    BmiConfiguration = 4,
    BmiData = 8,
    BmiUnavailable = 16
};
struct Bmi088Status {
    Bmi088Phase phase = Bmi088Phase::Off;
    uint32_t errors = 0, attempts = 0, read_failures = 0, last_sample_ms = 0;
    uint8_t accel_id = 0, gyro_id = 0;
};
// Sole task owner. begin() starts a finite state machine; poll() performs at most
// one short initialization operation or one bounded sensor read. No HAL_Delay.
class Bmi088 {
  public:
    void begin(uint32_t now_ms);
    bool poll(uint32_t now_ms, Bmi088Sample &out);
    const Bmi088Status &status() const {
        return status_;
    }

  private:
    Bmi088Status status_{};
    uint32_t due_ms_ = 0, sensor_time_ = 0, frames_ = 0;
    uint8_t config_index_ = 0;
    bool have_sensor_time_ = false;
    void fail(uint32_t now, uint32_t error);
};
} // namespace engineer
