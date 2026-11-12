#pragma once
#include <cmath>
#include <cstdint>

namespace engineer {
// Pure finite-pulse policy; task snapshots, TaskProtect and hardware adaptation
// belong to 3Task/yaw_service, not this device/control layer.
struct YawBenchRequest {
    uint32_t mode = 0; // 0=stop, 1=bounded velocity-PI pulse, never direct current.
    float speed_deg_s = 0;
    uint32_t duration_ms = 0, suspended_confirm = 0, request_seq = 0;
};
struct YawBenchHealth {
    bool ready = false, enabled = false, faulted = false, stationary = false, stop = false;
    uint32_t stop_epoch = 0;
};
struct YawBenchState {
    uint32_t consumed_seq = 0, result = 0, until_ms = 0;
    float speed_deg_s = 0;
    bool active = false;
};
class YawBenchGate {
  public:
    const YawBenchState &step(uint32_t now, const YawBenchRequest &r, const YawBenchHealth &h) {
        const bool was_active = s_.active;
        const bool stopped = initialized_ && h.stop_epoch != last_stop_epoch_;
        initialized_ = true;
        last_stop_epoch_ = h.stop_epoch;
        if (s_.active && (!h.ready || h.faulted || !h.enabled || h.stop || stopped ||
                          int32_t(now - s_.until_ms) >= 0)) {
            s_.active = false;
            s_.speed_deg_s = 0;
            s_.result = 4;
        }
        if (r.request_seq == s_.consumed_seq)
            return s_;
        s_.consumed_seq = r.request_seq;
        s_.active = false;
        s_.speed_deg_s = 0;
        s_.result = 2;
        if (r.mode == 0) {
            s_.result = 0;
            return s_;
        }
        if (r.mode != 1 || !std::isfinite(r.speed_deg_s) || std::fabs(r.speed_deg_s) > 5 ||
            !r.duration_ms || r.duration_ms > 500 || r.suspended_confirm != 0x59415753u)
            return s_;
        if (was_active || h.enabled || !h.ready || h.faulted || !h.stationary || h.stop ||
            stopped) {
            s_.result = 3;
            return s_;
        }
        s_.active = true;
        s_.speed_deg_s = r.speed_deg_s;
        s_.until_ms = now + r.duration_ms;
        s_.result = 1;
        return s_;
    }
    const YawBenchState &status() const {
        return s_;
    }

  private:
    YawBenchState s_{};
    uint32_t last_stop_epoch_ = 0;
    bool initialized_ = false;
};
} // namespace engineer
