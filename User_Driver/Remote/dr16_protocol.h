#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
enum { DR16_FRAME_SIZE = 18, DR16_SWITCH_UP = 1, DR16_SWITCH_DOWN = 2, DR16_SWITCH_MID = 3 };
typedef struct {
    int16_t channel[5];             // 原码减1024，四摇杆轴通常在[-660,660]
    uint8_t switches[2];           // 物理左右归属由OperatorConfig配置，实机验证
    int16_t mouse_x, mouse_y, mouse_z;
    uint8_t mouse_left, mouse_right;
    uint16_t keyboard;
} DR16Frame;
// 无CRC的DBUS报文只接受完整18字节及合理的摇杆/拨杆值，失败不修改out。
int DR16_Decode(const uint8_t *bytes, uint16_t length, DR16Frame *out);
#ifdef __cplusplus
}
#endif
