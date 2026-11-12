#include "Remote/operator_control.h"
namespace engineer {
const OperatorConfig kOperatorConfig = {
    false, // TODO: 通道/方向全部实物核对后再设true；当前仅确认左右拨杆索引
    1,
    0, // 实物确认：左拨杆=switches[1]，右拨杆=switches[0]
    3,
    2,
    1,
    0, // 左上下、左左右、右上下、右左右
    {1, 1, 1, 1},
    0.06f,
    100,
    100,
    300,
    0.0f,
    45.0f,  // 32T输出轮角；旧M2006轴侧60deg折算为45deg，仍需实测
    150.0f, // 保持电流上限占位，非夹持力标定值
    2000    // 闭合最多推进2秒，随后停止增加目标并降低电流上限
};
}
