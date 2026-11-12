#pragma once
#include <stdint.h>

// Only for the separate, one-motor CAN commissioning image. No motion commands.
namespace dmconfig {
enum Result : uint32_t {
    Idle,
    Reading,
    ReadOk,
    Applying,
    Applied,
    Saving,
    SaveAcknowledged,
    Cancelled,
    InvalidArgument,
    NeedRead,
    NeedApply,
    Timeout,
    VerifyFailed,
    UnsupportedMode,
    NotDisabled,
    TransportError,
    BusyRejected
};
struct Frame {
    uint16_t id;
    uint8_t len;
    uint8_t data[8];
};
struct Status {
    uint32_t result = Idle, busy = 0;
    uint32_t can_id = 0, mst_id = 0, sw_ver = 0, ctrl_mode = 0;
};

class Configurator {
  public:
    // 0 idle; 1 read source; 2 apply axis 1/2 IDs in RAM; 3 save once; 4 cancel.
    // Save uses the internally verified new address, not source_id/target_axis.
    void command(uint32_t command, uint32_t source_id, uint32_t target_axis, uint32_t now);
    bool step(uint32_t now, Frame &out);
    void receive(uint16_t id, const uint8_t *data, uint8_t len, uint32_t received_ms);
    void transport_fault();
    const Status &status() const {
        return status_;
    }

  private:
    enum Phase : uint8_t {
        None,
        ReadEsc,
        ReadMst,
        ReadFw,
        ReadMode,
        Disable,
        WriteMst,
        DelayMst,
        CheckMst,
        WriteEsc,
        DelayEsc,
        CheckEsc,
        FinalMst,
        Save,
        SaveDelay
    };
    Status status_{}, expected_{};
    Phase phase_ = None;
    uint8_t operation_ = 0, source_ = 0, target_ = 0;
    bool sent_ = false, read_valid_ = false, applied_ = false;
    uint32_t sent_ms_ = 0, phase_ms_ = 0;
    void phase(Phase next, uint32_t now);
    void finish(Result result);
};
} // namespace dmconfig
