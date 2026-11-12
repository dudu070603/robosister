#pragma once
#include <stdint.h>

#ifdef __cplusplus
#include "Arm/dm_id_config.h"
extern "C" {
#endif

typedef struct {
    uint32_t source_id;   // 当前电机ESC_ID；默认1，不代表已探测到实物
    uint32_t target_axis; // 1 -> 1/0x11；2 -> 2/0x12
    uint32_t command;     // 最后写入；任务读取后清零，0不执行操作
} DmIdConfigRequest;

extern volatile DmIdConfigRequest g_dm_config_request;
void dm_id_config_task(void *argument);

#ifdef __cplusplus
extern dmconfig::Status g_dm_config_status; // 只读快照，供现有调试工具观察
}
#endif
