#pragma once
#include "Arm/arm_control.h"
void Robot_Execute(engineer::ArmControl &arm, uint32_t now_ms, bool force_stop);
void Robot_ReportArmState(const engineer::Status &state);
