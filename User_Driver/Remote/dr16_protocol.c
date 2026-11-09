#include "dr16_protocol.h"
#include <stddef.h>

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)p[1] << 8); }
int DR16_Decode(const uint8_t *b, uint16_t n, DR16Frame *out) {
    if (!b || !out || n != DR16_FRAME_SIZE) return 0;
    DR16Frame f = {0};
    f.channel[0] = (int16_t)((b[0] | (uint16_t)b[1] << 8) & 0x7ff) - 1024;
    f.channel[1] = (int16_t)((b[1] >> 3 | (uint16_t)b[2] << 5) & 0x7ff) - 1024;
    f.channel[2] = (int16_t)((b[2] >> 6 | (uint16_t)b[3] << 2 | (uint16_t)b[4] << 10) & 0x7ff) - 1024;
    f.channel[3] = (int16_t)((b[4] >> 1 | (uint16_t)b[5] << 7) & 0x7ff) - 1024;
    f.channel[4] = (int16_t)(le16(b + 16) & 0x7ff) - 1024;
    f.switches[0] = (b[5] >> 4) & 3;
    f.switches[1] = (b[5] >> 6) & 3;
    for (unsigned i = 0; i < 4; ++i) if (f.channel[i] < -700 || f.channel[i] > 700) return 0;
    for (unsigned i = 0; i < 2; ++i) if (!f.switches[i] || f.switches[i] > 3) return 0;
    if (b[12] > 1 || b[13] > 1) return 0;
    f.mouse_x = (int16_t)le16(b + 6); f.mouse_y = (int16_t)le16(b + 8); f.mouse_z = (int16_t)le16(b + 10);
    f.mouse_left = b[12]; f.mouse_right = b[13]; f.keyboard = le16(b + 14);
    *out = f;
    return 1;
}
