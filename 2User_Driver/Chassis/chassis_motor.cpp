#include "Chassis/chassis_motor.h"
#include <cmath>
#include <cstring>
namespace engineer {
bool ChassisMotor::receive(const uint8_t *d, uint8_t len, uint32_t ms) {
    if (!d || len != 8) {
        return false;
    }
    const auto encoder = uint16_t(uint16_t(d[0]) << 8 | d[1]);
    if (encoder > 8191) {
        return false;
    }
    status_.encoder = encoder;
    status_.rotor_rpm = int16_t(uint16_t(d[2]) << 8 | d[3]);
    status_.current_ma = int16_t(uint16_t(d[4]) << 8 | d[5]) * (20000.0f / 16384.0f);
    status_.temperature_c = d[6];
    status_.received_ms = ms;
    ++status_.frames;
    return true;
}
void ChassisMotor::pack_current(float l, float r, const uint8_t id[2], const int8_t direction[2],
                                uint8_t d[8], float actual_ma[2]) {
    std::memset(d, 0, 8);
    const float ma[2] = {l, r};
    for (unsigned i = 0; i < 2; ++i) {
        if (id[i] < 1 || id[i] > 4 || (direction[i] != 1 && direction[i] != -1) ||
            !std::isfinite(ma[i])) {
            continue;
        }
        const float limited = std::fmax(-20000.0f, std::fmin(20000.0f, ma[i]));
        const int16_t raw = int16_t(limited * direction[i] * (16384.0f / 20000.0f));
        d[2 * (id[i] - 1)] = uint16_t(raw) >> 8;
        d[2 * (id[i] - 1) + 1] = uint16_t(raw) & 255;
    }
    if (!actual_ma) {
        return;
    }
    // Decode the completed frame, including slot selection, sign and integer
    // truncation. The owner can feed these values directly to PI anti-windup.
    for (unsigned i = 0; i < 2; ++i) {
        actual_ma[i] = 0;
        if (id[i] < 1 || id[i] > 4 || (direction[i] != 1 && direction[i] != -1)) {
            continue;
        }
        const unsigned slot = 2 * (id[i] - 1);
        const int16_t raw = int16_t(uint16_t(d[slot]) << 8 | uint16_t(d[slot + 1]));
        actual_ma[i] = raw * (20000.0f / 16384.0f) * direction[i];
    }
}
} // namespace engineer
