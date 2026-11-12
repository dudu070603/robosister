#include "Arm/dm_id_config.h"

namespace dmconfig {
namespace {
constexpr uint32_t kReplyTimeout = 200, kWriteSettle = 20, kSaveSettle = 30;
uint32_t value(const uint8_t *d) {
    return uint32_t(d[4]) | (uint32_t(d[5]) << 8) | (uint32_t(d[6]) << 16) | (uint32_t(d[7]) << 24);
}
// Parameters echo the full ESC address; they are not motor-state feedback.
bool parameter_reply(const uint8_t *d) {
    return (d[2] == 0x33 || d[2] == 0x55 || (d[2] == 0xaa && d[3] == 1));
}
} // namespace

void Configurator::phase(Phase next, uint32_t now) {
    phase_ = next;
    phase_ms_ = now;
    sent_ = false;
}
void Configurator::finish(Result result) {
    phase_ = None;
    sent_ = false;
    status_.result = result;
    status_.busy = 0;
    read_valid_ = result == ReadOk || result == Applied || result == SaveAcknowledged;
    applied_ = result == Applied;
}
void Configurator::transport_fault() {
    finish(TransportError);
}

void Configurator::command(uint32_t cmd, uint32_t source, uint32_t target, uint32_t now) {
    if (!cmd)
        return;
    if (cmd == 4) {
        finish(Cancelled);
        return;
    }
    // A new command never replaces an in-flight write/save with another write.
    if (status_.busy) {
        finish(BusyRejected);
        return;
    }
    if (cmd > 3 || (cmd != 3 && source > 15) || (cmd == 2 && (target < 1 || target > 2))) {
        finish(InvalidArgument);
        return;
    }
    if (cmd == 2 && (!read_valid_ || source != status_.can_id)) {
        finish(NeedRead);
        return;
    }
    if (cmd == 3 && !applied_) {
        finish(NeedApply);
        return;
    }
    expected_ = status_;
    source_ = uint8_t(cmd == 3 ? status_.can_id : source);
    target_ = uint8_t(target);
    operation_ = uint8_t(cmd);
    read_valid_ = applied_ = false; // Save permission is consumed before any traffic.
    status_.busy = 1;
    status_.result = cmd == 1 ? Reading : cmd == 2 ? Applying : Saving;
    phase(ReadEsc, now);
}

bool Configurator::step(uint32_t now, Frame &out) {
    if (phase_ == None)
        return false;
    if (phase_ == SaveDelay) {
        if (uint32_t(now - sent_ms_) >= kSaveSettle)
            finish(SaveAcknowledged);
        return false;
    }
    if (!sent_ && uint32_t(now - phase_ms_) >= kReplyTimeout) {
        finish(Timeout);
        return false;
    }
    if (phase_ == DelayMst || phase_ == DelayEsc) {
        if (uint32_t(now - phase_ms_) < kWriteSettle)
            return false;
        phase(phase_ == DelayMst ? CheckMst : CheckEsc, now);
    }
    if (sent_) {
        if (uint32_t(now - sent_ms_) >= kReplyTimeout)
            finish(Timeout);
        return false;
    }
    out = {};
    out.id = 0x7ff;
    out.len = 8;
    out.data[0] = source_;
    out.data[2] = 0x33;
    switch (phase_) {
    case ReadEsc:
    case CheckEsc:
        out.data[3] = 8;
        break;
    case ReadMst:
    case CheckMst:
    case FinalMst:
        out.data[3] = 7;
        break;
    case ReadFw:
        out.data[3] = 0x0e;
        break;
    case ReadMode:
        out.data[3] = 0x0a;
        break;
    case Disable:
        out.id = uint16_t(source_ + (status_.ctrl_mode - 1) * 0x100);
        for (unsigned i = 0; i < 7; ++i)
            out.data[i] = 0xff;
        out.data[7] = 0xfd;
        break;
    case WriteMst:
        out.data[2] = 0x55;
        out.data[3] = 7;
        out.data[4] = uint8_t(0x10 + target_);
        phase(DelayMst, now);
        return true;
    case WriteEsc:
        out.data[2] = 0x55;
        out.data[3] = 8;
        out.data[4] = target_;
        source_ =
            target_; // ACK may use either address; only a fresh read at the new one verifies it.
        phase(DelayEsc, now);
        return true;
    case Save:
        out.data[2] = 0xaa;
        out.data[3] = 1; // J4310P model manual, not generic SDK's 0x00.
        break;
    default:
        return false;
    }
    sent_ = true;
    sent_ms_ = now;
    return true;
}

void Configurator::receive(uint16_t id, const uint8_t *d, uint8_t len, uint32_t now) {
    if (!d || !sent_ || id > 0x7ff || int32_t(now - sent_ms_) < 0 ||
        uint32_t(now - sent_ms_) >= kReplyTimeout)
        return;
    if (phase_ == Save) {
        if ((len == 4 || len == 8) && id == status_.mst_id && d[0] == source_ && !d[1] &&
            d[2] == 0xaa && d[3] == 1) {
            // Flash may still be busy when an early ACK arrives. Never send a second save.
            phase_ = SaveDelay;
        }
        return;
    }
    if (len != 8)
        return;
    if (phase_ == Disable) {
        if (id != status_.mst_id || (d[0] & 0x0f) != source_)
            return;
        if (d[0] == source_ && !d[1] && parameter_reply(d))
            return;
        if (d[0] >> 4) {
            finish(NotDisabled);
            return;
        }
        if (operation_ == 1)
            finish(ReadOk);
        else
            phase(operation_ == 2 ? WriteMst : Save, now);
        return;
    }
    if (d[0] != source_ || d[1] || d[2] != 0x33)
        return;
    uint8_t rid = 0;
    switch (phase_) {
    case ReadEsc:
    case CheckEsc:
        rid = 8;
        break;
    case ReadMst:
    case CheckMst:
    case FinalMst:
        rid = 7;
        break;
    case ReadFw:
        rid = 0x0e;
        break;
    case ReadMode:
        rid = 0x0a;
        break;
    default:
        return;
    }
    if (d[3] != rid)
        return;
    const uint32_t v = value(d);
    if (phase_ == ReadEsc) {
        if (v != source_) {
            finish(VerifyFailed);
            return;
        }
        status_.can_id = v;
        status_.mst_id = id;
        phase(ReadMst, now);
        return;
    }
    if (phase_ == CheckMst) {
        if (v != 0x10u + target_ || id != v) {
            finish(VerifyFailed);
            return;
        }
        status_.mst_id = v;
        phase(WriteEsc, now);
        return;
    }
    if (id != status_.mst_id)
        return;
    switch (phase_) {
    case ReadMst:
        if (v != id) {
            finish(VerifyFailed);
            return;
        }
        phase(ReadFw, now);
        break;
    case ReadFw:
        status_.sw_ver = v;
        phase(ReadMode, now);
        break;
    case ReadMode:
        if (v < 1 || v > 4) {
            finish(UnsupportedMode);
            return;
        }
        status_.ctrl_mode = v;
        if (operation_ != 1 &&
            (status_.can_id != expected_.can_id || status_.mst_id != expected_.mst_id ||
             status_.sw_ver != expected_.sw_ver || status_.ctrl_mode != expected_.ctrl_mode)) {
            finish(VerifyFailed);
            return;
        }
        phase(Disable, now);
        break;
    case CheckEsc:
        if (v != source_) {
            finish(VerifyFailed);
            return;
        }
        status_.can_id = v;
        phase(FinalMst, now);
        break;
    case FinalMst:
        if (v != status_.mst_id) {
            finish(VerifyFailed);
            return;
        }
        finish(Applied);
        break;
    default:
        break;
    }
}
} // namespace dmconfig
