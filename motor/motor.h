#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

// 根据你 32MHz, 15kHz 的设置，Period Count 是 2132
#define MOTOR_MAX_PWM    350

void Motor_Init(void);
void Motor_Set_PWM(int32_t left_pwm, int32_t right_pwm);
void Motor_Left_Set(int32_t pwm);
void Motor_Right_Set(int32_t pwm);
void Motor_Task_Start_Init(int32_t left_start_pwm, int32_t right_start_pwm);

#endif