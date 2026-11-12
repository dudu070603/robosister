#pragma once
#include <cstdint>
namespace engineer {
struct PowerMeterStatus {
    float power_w = 0, energy_j = 0;
    uint32_t received_ms = 0, frames = 0;
};
class PowerMeter {
  public:
    bool receive(const uint8_t *data, uint8_t len, uint32_t ms);
    const PowerMeterStatus &status() const {
        return status_;
    }

  private:
    PowerMeterStatus status_{};
};
} // namespace engineer
