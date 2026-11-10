#include "dm_id_config.h"
#include <cstring>
#include <initializer_list>
using namespace dmconfig;
extern "C" volatile unsigned test_checks;
#define CHECK(x) do { ++test_checks; if (!(x)) return __LINE__; } while (0)

namespace {
bool read_frame(const Frame &f, unsigned source, unsigned rid) {
    return f.id == 0x7ff && f.len == 8 && f.data[0] == source && !f.data[1] &&
        f.data[2] == 0x33 && f.data[3] == rid && !f.data[4] && !f.data[5] && !f.data[6] && !f.data[7];
}
void reply(Configurator &c, unsigned src, unsigned mst, unsigned rid, uint32_t v, uint32_t t) {
    uint8_t d[8] = {uint8_t(src), 0, 0x33, uint8_t(rid),
        uint8_t(v), uint8_t(v >> 8), uint8_t(v >> 16), uint8_t(v >> 24)};
    c.receive(uint16_t(mst), d, 8, t);
}
bool read_and_disable(Configurator &c, uint32_t &t, unsigned src, unsigned mst, unsigned mode = 1) {
    Frame f{};
    const unsigned rid[] = {8, 7, 0x0e, 0x0a}, values[] = {src, mst, 0x12345678, mode};
    for (unsigned i = 0; i < 4; ++i) {
        if (!c.step(++t, f) || !read_frame(f, src, rid[i])) return false;
        reply(c, src, mst, rid[i], values[i], ++t);
    }
    if (mode < 1 || mode > 4) return c.status().result == UnsupportedMode;
    if (c.status().result == VerifyFailed) return true;
    if (!c.step(++t, f) || f.id != src + (mode - 1) * 0x100 || f.len != 8 || f.data[7] != 0xfd) return false;
    for (unsigned i = 0; i < 7; ++i) if (f.data[i] != 0xff) return false;
    uint8_t off[8] = {uint8_t(src), 0x80, 0, 0x80, 0, 0, 25, 25};
    c.receive(uint16_t(mst), off, 8, ++t);
    return true;
}
bool begin_read(Configurator &c, uint32_t &t, unsigned src = 1, unsigned mst = 0, unsigned mode = 1) {
    c.command(1, src, 1, t);
    return read_and_disable(c, t, src, mst, mode) && c.status().result == ReadOk;
}
bool apply(Configurator &c, uint32_t &t, unsigned src = 1, unsigned mst = 0, unsigned target = 2) {
    c.command(2, src, target, ++t);
    if (!read_and_disable(c, t, src, mst)) return false;
    Frame f{};
    if (!c.step(++t, f) || f.id != 0x7ff || f.data[2] != 0x55 || f.data[3] != 7 ||
        f.data[0] != src || f.data[4] != 0x10 + target) return false;
    if (c.step(t + 19, f)) return false;
    t += 20;
    if (!c.step(t, f) || !read_frame(f, src, 7)) return false;
    reply(c, src, 0x10 + target, 7, 0x10 + target, ++t);
    if (!c.step(++t, f) || f.id != 0x7ff || f.data[2] != 0x55 || f.data[3] != 8 ||
        f.data[0] != src || f.data[4] != target) return false;
    if (c.step(t + 19, f)) return false;
    t += 20;
    if (!c.step(t, f) || !read_frame(f, target, 8)) return false;
    reply(c, target, 0x10 + target, 8, target, ++t);
    if (!c.step(++t, f) || !read_frame(f, target, 7)) return false;
    reply(c, target, 0x10 + target, 7, 0x10 + target, ++t);
    return c.status().result == Applied && !c.status().busy;
}
}

int test_dm_id_config() {
    Frame f{};
    { // Startup, idle, and invalid inputs cannot produce any frame, especially writes.
        Configurator c;
        for (unsigned t = 0; t < 1000; ++t) { c.command(0, 0, 0, t); CHECK(!c.step(t, f)); }
        CHECK(c.status().result == Idle && !c.status().busy);
        c.command(2, 1, 1, 1000); CHECK(c.status().result == NeedRead && !c.step(1000, f));
        c.command(3, 1, 1, 1000); CHECK(c.status().result == NeedApply && !c.step(1000, f));
        for (auto cmd : {1u, 2u}) {
            c.command(cmd, 16, 1, 1000); CHECK(c.status().result == InvalidArgument && !c.step(1000, f));
        }
        c.command(2, 1, 0, 1000); CHECK(c.status().result == InvalidArgument);
        c.command(99, 1, 1, 1000); CHECK(c.status().result == InvalidArgument);
    }
    { // Unknown feedback CAN ID, including zero; all supported modes issue only FD at their own offset.
        for (unsigned mode = 1; mode <= 4; ++mode) {
            Configurator c; uint32_t t = 0;
            CHECK(begin_read(c, t, mode == 4 ? 0 : 15, mode == 1 ? 0 : 0x321, mode));
            CHECK(c.status().mst_id == (mode == 1 ? 0u : 0x321u));
            CHECK(c.status().sw_ver == 0x12345678 && c.status().ctrl_mode == mode);
            CHECK(!c.step(t + 1000, f));
        }
        Configurator c; uint32_t t = 0; c.command(1, 1, 1, 0);
        CHECK(read_and_disable(c, t, 1, 0, 0)); CHECK(!c.step(++t, f));
    }
    { // No stale/malformed/read-ACK impostor can progress a transaction.
        Configurator c; c.command(1, 1, 1, 100); CHECK(c.step(100, f));
        uint8_t d[8] = {1, 0, 0x33, 8, 1, 0, 0, 0};
        c.receive(0, d, 7, 101); c.receive(0, d, 8, 99);
        c.receive(0x800, d, 8, 101);
        d[0] = 2; c.receive(0, d, 8, 101); d[0] = 1;
        d[3] = 7; c.receive(0, d, 8, 101); d[3] = 8;
        d[2] = 0x55; c.receive(0, d, 8, 101); d[2] = 0x33;
        CHECK(!c.step(102, f) && c.status().busy);
        c.receive(0x77, d, 8, 103); CHECK(c.step(104, f) && read_frame(f, 1, 7));
        reply(c, 1, 0x76, 7, 0x76, 105); CHECK(!c.step(106, f)); // wrong responder after identity discovery
        reply(c, 1, 0x77, 7, 0x77, 107); CHECK(c.step(108, f));
        reply(c, 1, 0x77, 0x0e, 123, 109); CHECK(c.step(110, f));
        reply(c, 1, 0x77, 0x0a, 1, 111); CHECK(c.step(112, f) && f.data[7] == 0xfd);
        reply(c, 1, 0x77, 0x0a, 1, 113); // parameter ACK low first byte is not proof of disabled state
        uint8_t saved[8] = {1, 0, 0xaa, 1, 0, 0, 0, 0}; c.receive(0x77, saved, 8, 114);
        CHECK(c.status().busy && c.status().result == Reading);
        uint8_t enabled[8] = {0x11, 0x80, 0, 0x80, 0, 0, 25, 25}; c.receive(0x77, enabled, 8, 115);
        CHECK(c.status().result == NotDisabled && !c.status().busy && !c.step(116, f));
    }
    { // Source must be re-confirmed before writes, not trusted from a previous read.
        Configurator c; uint32_t t = 0; CHECK(begin_read(c, t));
        c.command(2, 2, 2, ++t); CHECK(c.status().result == NeedRead);
        CHECK(begin_read(c, t)); c.command(2, 1, 2, ++t);
        CHECK(read_and_disable(c, t, 1, 0x22)); // changed responder rejected before FD/write
        CHECK(c.status().result == VerifyFailed && !c.step(++t, f));
    }
    { // Apply writes only two ID registers; independent reads at new addresses are mandatory.
        Configurator c; uint32_t t = 0; CHECK(begin_read(c, t)); CHECK(apply(c, t));
        CHECK(c.status().can_id == 2 && c.status().mst_id == 0x12);
        for (unsigned i = 0; i < 100; ++i) CHECK(!c.step(++t, f)); // never automatically save
        c.command(3, 1, 1, ++t); // deliberately old UI source; save must use internally verified address2
        CHECK(read_and_disable(c, t, 2, 0x12));
        CHECK(c.step(++t, f) && f.id == 0x7ff && f.data[0] == 2 && f.data[2] == 0xaa && f.data[3] == 1);
        const uint32_t saved_at = t;
        uint8_t ack[8] = {2, 0, 0xaa, 0, 0, 0, 0, 0};
        c.receive(0x12, ack, 8, t + 1); CHECK(c.status().busy); // generic SDK 0x00 is not accepted
        ack[3] = 1; c.receive(0x11, ack, 8, t + 2); CHECK(c.status().busy);
        c.receive(0x12, ack, 4, t + 3);
        for (t = saved_at + 4; t < saved_at + 30; ++t) CHECK(!c.step(t, f) && c.status().busy);
        CHECK(!c.step(t, f) && c.status().result == SaveAcknowledged);
        c.command(3, 2, 2, ++t); CHECK(c.status().result == NeedApply && !c.step(t, f));
    }
    { // Missing save ACK is not success; no retry after a timeout or transport failure.
        for (unsigned scenario = 0; scenario < 2; ++scenario) {
            Configurator c; uint32_t t = 0; CHECK(begin_read(c, t)); CHECK(apply(c, t));
            c.command(3, 0, 0, ++t); CHECK(read_and_disable(c, t, 2, 0x12)); CHECK(c.step(++t, f));
            if (scenario) c.transport_fault(); else CHECK(!c.step(t + 200, f));
            CHECK(c.status().result == (scenario ? TransportError : Timeout));
            for (unsigned i = 0; i < 100; ++i) CHECK(!c.step(t + 201 + i, f));
            c.command(3, 2, 2, t + 400); CHECK(c.status().result == NeedApply);
        }
    }
    { // Busy commands cancel rather than re-enter; explicit cancel and late feedback cannot resume.
        Configurator c; c.command(1, 1, 1, 0); CHECK(c.step(0, f));
        c.command(2, 1, 2, 1); CHECK(c.status().result == BusyRejected && !c.status().busy);
        reply(c, 1, 0, 8, 1, 2); CHECK(!c.step(3, f));
        c.command(1, 1, 1, 4); CHECK(c.step(4, f)); c.command(4, 0, 0, 5);
        CHECK(c.status().result == Cancelled); reply(c, 1, 0, 8, 1, 6); CHECK(!c.step(7, f));
        c.command(1, 1, 1, 8); CHECK(!c.step(208, f) && c.status().result == Timeout);
        c.command(1, 1, 1, 0xfffffff0); CHECK(c.step(0xfffffff0, f));
        reply(c, 1, 0, 8, 1, 4); CHECK(c.step(5, f) && read_frame(f, 1, 7)); // tick wrap
    }
    { // Wrong read-back value is rejected, and neither an ACK nor a stale frame opens saving.
        Configurator c; uint32_t t = 0; CHECK(begin_read(c, t));
        c.command(2, 1, 2, ++t); CHECK(read_and_disable(c, t, 1, 0)); CHECK(c.step(++t, f));
        uint8_t ack[8] = {1, 0, 0x55, 7, 0x12, 0, 0, 0};
        c.receive(0x12, ack, 8, ++t); CHECK(c.status().result == Applying);
        t += 20; CHECK(c.step(t, f) && read_frame(f, 1, 7));
        reply(c, 1, 0x12, 7, 0x11, ++t); CHECK(c.status().result == VerifyFailed);
        c.command(3, 1, 2, ++t); CHECK(c.status().result == NeedApply && !c.step(t, f));
    }
    { // Pausing a task must not turn a stale disabled confirmation into a late write.
        for (unsigned scenario = 0; scenario < 2; ++scenario) {
            Configurator c; uint32_t t = 0; CHECK(begin_read(c, t));
            c.command(2, 1, 2, ++t); CHECK(read_and_disable(c, t, 1, 0));
            if (scenario) CHECK(c.step(++t, f) && f.data[2] == 0x55 && f.data[3] == 7);
            CHECK(!c.step(t + 200, f) && c.status().result == Timeout);
            CHECK(!c.step(t + 201, f));
            c.command(3, 1, 2, t + 202); CHECK(c.status().result == NeedApply);
        }
    }
    { // ESC zero is valid for discovery/disable; the first axis receives 1/0x11.
        Configurator c; uint32_t t = 0; CHECK(begin_read(c, t, 0, 0));
        CHECK(apply(c, t, 0, 0, 1));
        CHECK(c.status().can_id == 1 && c.status().mst_id == 0x11);
    }
    return 0;
}
