#include "Power/power_meter.h"
namespace engineer {
bool PowerMeter::receive(const uint8_t *d, uint8_t len, uint32_t ms) {
    if (!d || len != 4)
        return false;
    status_.power_w = int16_t(uint16_t(d[0]) | uint16_t(d[1]) << 8) * .1f;
    status_.energy_j = int16_t(uint16_t(d[2]) | uint16_t(d[3]) << 8) * .1f;
    status_.received_ms = ms;
    ++status_.frames;
    return true;
}
} // namespace engineer
