#ifndef _MAIN_H_
#define _MAIN_H_

#include "clock.h"
#include "interrupt.h"
#include "pid.h"
#include "motor.h"
//#include "comm.h"
#include "comm2.h"
#include "pid.h"
#include "stdlib.h"
#include "hwt101.h"
//#include "oled.h"
// 1. 定义枚举类型
typedef enum {
    STATE_IDLE = 0,
    STATE_TASK1 = 1,
    STATE_TASK2 = 2,
    STATE_TASK3 = 3,
    STATE_TASK4 = 4
} SystemState_t;

// 2. 声明外部变量
extern  volatile uint8_t stable_line_flag ;    // 防抖之后稳定标志
// extern  uint16_t stable_filter_cnt ;
extern  uint16_t is_notonblakline ;
extern  volatile uint8_t line_exist;
extern  uint8_t cross_point_cnt;
extern  volatile SystemState_t g_systemState;
extern volatile uint8_t line_exist;
extern volatile int16_t BASE_SPEED;
extern volatile uint8_t g_task_ready_flag;
extern pid_t trace_pid;
extern pid_t angle;
// extern  float yaw;
 extern volatile float yaw ;
//extern  uint8_t g_send_cmd_count;
//extern volatile uint8_t g_trace_error_count;
extern volatile float_t a;
extern volatile long long aaa;
extern  volatile  int  se;
extern volatile int8_t gray_error;
extern  volatile uint8_t use_gyro_pid;
extern volatile uint8_t corner_total_cnt;
extern volatile uint8_t cycle_flag ;
extern volatile uint8_t uart_rx_allow ;
extern volatile uint8_t angle_init_flag ;
extern  volatile int16_t corner_detect_cnt;
extern  volatile uint8_t corner_detectcnt;
//extern volatile SystemState_t g_systemState;


/*include "ultrasonic_capture.h"
#include "ultrasonic_gpio.h"
#include "bno08x_uart_rvc.h"
#include "wit.h"
#include "vl53l0x.h"
#include "lsm6dsv16x.h"
*/
#endif  /* #ifndef _MAIN_H_ */
