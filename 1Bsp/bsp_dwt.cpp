#include "bsp_dwt.h"
#include "stm32f4xx_hal.h"
namespace {
uint32_t core_hz_ = 0;
}
extern "C" int BSP_DWT_Init(uint32_t hz) {
    if (hz < 1000000 || (core_hz_ && core_hz_ != hz))
        return 0;
    if (!core_hz_) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        __DSB();
        __ISB();
        core_hz_ = hz;
    }
    return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0;
}
extern "C" uint32_t BSP_DWT_Cycles() {
    return DWT->CYCCNT;
}
extern "C" uint32_t BSP_DWT_CoreHz() {
    return core_hz_;
}
extern "C" float BSP_DWT_ElapsedSeconds(uint32_t current, uint32_t previous) {
    return core_hz_ ? float(uint32_t(current - previous)) / float(core_hz_) : 0;
}
