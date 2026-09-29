#ifndef HWT101_H_
#define HWT101_H_

#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#ifdef __cplusplus
extern "C" {
#endif

/* HWT101 角度数据。单位使用 0.01 度，避免在 Cortex-M0+ 上引入浮点打印。 */
typedef struct {
    int16_t rawRoll;
    int16_t rawPitch;
    int16_t rawYaw;
    int32_t rollCentiDeg;
    int32_t pitchCentiDeg;
    int32_t yawCentiDeg;
} HWT101_Angle;

/* 使能 I2C0 中断；调用前必须已经执行 SYSCFG_DL_init()。 */
void HWT101_init(void);

/* 读取横滚、俯仰、偏航角。返回 false 表示 I2C 通信失败或参数无效。 */
bool HWT101_readAngles(HWT101_Angle *angle);

#ifdef __cplusplus
}
#endif

#endif /* HWT101_H_ */
