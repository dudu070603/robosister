#include "operator_control.h"
namespace engineer {
const OperatorConfig kOperatorConfig = {
    false, // TODO: 在DR16_GetStatus中核对通道/拨杆实物后设true
    0, 1,  // 左/右拨杆原码索引；按DJI示例注释暂定，不代替实物确认
    3, 2, 1, 0, // 左上下、左左右、右上下、右左右
    {1,1,1,1},
    0.06f, 100, 100, 300,
    0.0f, 45.0f, // 32T输出轮角；旧M2006轴侧60deg折算为45deg，仍需实测
    150.0f,      // 保持电流上限占位，非夹持力标定值
    2000         // 闭合最多推进2秒，随后停止增加目标并降低电流上限
};
}
