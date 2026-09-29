#ifndef __COMM2_H
#define __COMM2_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"
#include "main.h"

// 和原有代码统一串口硬件
#define UART_COMM2_INST      UART_1_INST
#define UART_COMM2_IRQN      UART_1_INST_INT_IRQN

// 缓存与传感器通道配置，和你STM32代码保持一致
#define RX_BUFFER_MAX_LEN  258
#define SENSOR_CH_NUM       8
#define CORNER_TRIG_CNT     30
#define CORNER_ERR_THRESHOLD    13

// 对外全局变量
extern uint8_t rx2_buffer[RX_BUFFER_MAX_LEN];
extern uint16_t rx2_idx;
extern uint8_t sensor_status[SENSOR_CH_NUM];
extern bool track_data_ready;
extern uint16_t is_notonblakline;

// 函数声明
void comm2_uart_init(void);
void comm2_send_start_cmd(void);
void Parse_Sensor_Data(void);
int8_t Get_Track_Error(void);
float_t Get_Target_Yaw(uint8_t state, float_t yaw);
#endif