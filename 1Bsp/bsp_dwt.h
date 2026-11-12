#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
// Initialize once after clocks stabilize, before IMU task start. Does not reset
// an already-running counter; a second call with the same clock is harmless.
int BSP_DWT_Init(uint32_t core_hz);
uint32_t BSP_DWT_Cycles(void);
uint32_t BSP_DWT_CoreHz(void);
// Unsigned subtraction handles one CYCCNT wrap. Call more often than 25 s at
// 168 MHz; no global 64-bit timeline and no floating-point wall-clock drift.
float BSP_DWT_ElapsedSeconds(uint32_t current, uint32_t previous);
#ifdef __cplusplus
}
#endif
