/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*----------------------------------- 1. 头文件引用区 -----------------------------------*/
#include "ti_msp_dl_config.h"
#include "main.h"
#define RELOAD_KEY_1   (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_24))
#define RELOAD_KEY_2   (DL_GPIO_readPins(GPIOA, DL_GPIO_PIN_25))
#define TRACE_ERROR_LIMIT   20
// 巡线PID参数宏定义
// #define TRACE_P_KP     12.0f
// #define TRACE_P_KI     0.02f
// #define TRACE_P_KD     0.035f
#define TRACE_P_KP     35.0f
#define TRACE_P_KI     0.02f
#define TRACE_P_KD     0.035f
 /*----------------------------------- 2. 全局变量定义区 -----------------------------------*/
// 【PID全局变量】
//volatile  int  se=12;
//volatile  long long  aaa=0;
//uint8_t g_send_cmd_count =4;//
volatile uint8_t cross_lock = 0;
#define CROSS_LOCK_TICK 100    // 定时器中断次数，根据你的中断周期调整
volatile uint8_t cross_lock_tick = 0;
volatile int8_t gray_error=0;
pid_t trace_pid;        // 灰度传感器巡线PID（核心）
pid_t angle;            // MPU航向角PID（转弯用
volatile uint8_t g_task_ready_flag=0;
//volatile uint8_t g_trace_error_count=0;
volatile int16_t BASE_SPEED =500;
//uint8_t g_send_cmd_count = 0;
// 注意：MSPM0按下通常返回0，不按下返回(1<<PIN_位)
volatile float_t a=0;
/*typedef enum {
    STATE_IDLE = 0,     // 待机：仅显示数据，不动作
    STATE_TASK1,
    STATE_TASK2,
    STATE_TASK3,
    STATE_TASK4
} SystemState_t;*/

volatile SystemState_t g_systemState = STATE_TASK4;
/*-----------------------------------  圈数以及标志位判断区-----------------------------------*/
volatile uint8_t corner_total_cnt= 0;      //已完成圈数
volatile uint8_t target_cycle = 0;     // 目标圈数
volatile uint8_t cycle_flag = 0;        // 单圈防重复计数标记
volatile uint8_t angle_init_flag = 1;//角度初始赋值标志位
volatile int16_t corner_detect_cnt = 0;
volatile uint8_t corner_detectcnt = 0;
uint8_t last_line_exist=0;
uint8_t cross_point_cnt=0;
// uint8_t change=0;
// 全局变量
int16_t Q=0;
HWT101_Angle hwt101_data;
volatile float yaw = 0.0f;

/*----------------------------------- 3. 函数声明区 -----------------------------------*/
 void ReloadKeyCheck(void);
 void ReloadKeyCheck_2(void);
 void comm_handle_direction_logic(void);
 void Control_Update_Trace_Feedback(int16_t deviation);

 /*----------------------------------- 4. 主函数 -----------------------------------*/
    int main(void)
    {
        SYSCFG_DL_init();
        SysTick_Init();
        comm2_uart_init();

        mspm0_delay_ms(10);


        mspm0_delay_ms(10);

        mspm0_delay_ms(10);
        HWT101_init();
        Interrupt_Init();
        mspm0_delay_ms(10);       // 小幅延时等待外设时钟稳定，可选加固

        //uart_rx_allow = 1;  // 全局中断就绪后，软件放行串口接收
        //NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        // 启动定时器计数
        //DL_TimerG_startCounter(TIMER_0_INST);
        Motor_Init();

        //comm2_uart_init();
        pid_init(&trace_pid, TRACE_P_KP ,  TRACE_P_KI,  TRACE_P_KD);   // 巡线PD（I=0，不抖动）
        // pid_init(&angle,     15.0f,  0.0f,  5.8f);   // 角度PID（弱积分，精准）
         pid_init(&angle,     6.0f,  0.05f,  20.2f);   // 角度PID（弱积分，精准）

        NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
        // 启动定时器计数
        DL_TimerG_startCounter(TIMER_0_INST);
        //DL_UART_Main_transmitData(UART_1_INST, 1);
    while (1)
    {   //Get_Track_Error();
                if(HWT101_readAngles(&hwt101_data))
    {
        yaw = hwt101_data.yawCentiDeg / 100.0f;
    }

                if(g_task_ready_flag ==1 )
            {


                    switch(g_systemState)
                    {
                        case STATE_TASK1:

                            if(line_exist){
                                g_task_ready_flag =0;
                            }
                            break;
                        case STATE_TASK2:
                              // 交汇点计数达到4次，停止任务
                        if(cross_point_cnt >= 4)
                        {
                            g_task_ready_flag = 0;
                            Motor_Set_PWM(0, 0);
                        }
                            break;
                        case STATE_TASK3:

                             if(cross_point_cnt >= 4)
                        {
                            g_task_ready_flag = 0;
                            Motor_Set_PWM(0, 0);
                        }
                            break;
                        case STATE_TASK4:
                          if(cross_point_cnt >= 16)
                        {
                            g_task_ready_flag = 0;
                            Motor_Set_PWM(0, 0);
                        }
                            break;
                        case STATE_IDLE:
                        default:
                            Motor_Set_PWM(0, 0);
                            break;
                    }

                }


                else{


                    Motor_Set_PWM(0,0 );
                    //Buzzer_TaskFinish();
                    //delay_ms(30);
                }

                //Get_Track_Error();





    }
    }

/*----------------------------------- 5. 函数实现 -----------------------------------*/

void TIMER_0_INST_IRQHandler(void)
{        // 仅中断内读取一次，全局yaw实时刷新

    //
    // --- 第一部分：按键检测 (仅在任务未启动时) ---
    if(g_task_ready_flag == 0)
    {
        ReloadKeyCheck();
        ReloadKeyCheck_2();
        // 任务未启动，电机强制停止
        Motor_Set_PWM(0, 0);
    }
    else // --- 第二部分：任务执行逻辑 (g_task_ready_flag == 1) ---
    {
switch(g_systemState)
{
    case STATE_TASK1:
        Control_Execute_Angle();
        break;

    case STATE_TASK2:
        if(last_line_exist != line_exist)
        {
            cross_point_cnt++;
            angle_init_flag = 1;
        }
        // 更新【稳定状态】缓存
        last_line_exist = line_exist;

        if(line_exist)
        {
            Control_Execute_Trace();
        }
        else
        {
            Control_Execute_Angle();
        }
        break;

    case STATE_TASK3:
        if(cross_lock)
    {
        cross_lock_tick--;
        if(cross_lock_tick == 0)
        {
            cross_lock = 0;
        }
    }
        if(!cross_lock&&last_line_exist != line_exist)
        {
            cross_point_cnt++;
            angle_init_flag = 1;
        cross_lock = 1;
        cross_lock_tick = CROSS_LOCK_TICK;
        }
        // 更新【稳定状态】缓存
        last_line_exist = line_exist;

        if(line_exist)
        {
            Control_Execute_Trace();
        }
        else
        {
            Control_Execute_Angle();
        }
        break;

    case STATE_TASK4:
        if(cross_lock)
    {
        cross_lock_tick--;
        if(cross_lock_tick == 0)
        {
            cross_lock = 0;
        }
    }
        if(!cross_lock&&last_line_exist != line_exist)
        {
            cross_point_cnt++;
            angle_init_flag = 1;
        cross_lock = 1;
        cross_lock_tick = CROSS_LOCK_TICK;
        }
        // 更新【稳定状态】缓存
        last_line_exist = line_exist;

        if(line_exist)
        {
            Control_Execute_Trace();
        }
        else
        {
            Control_Execute_Angle();
        }
        break;
    case STATE_IDLE:
    default:
        Motor_Set_PWM(0, 0);
        break;
}
    }
}
void ReloadKeyCheck(void)
{
    volatile static uint16_t DownTime1 = 0;        // 检测状态的时间（定时器的触发次数）
    volatile static uint8_t KEY_STATE = 1; // 按键的当前状态
    if( KEY_STATE!=0 )     // 状态1：按键松开状态
    {
        if(RELOAD_KEY_2 !=0 )           // 稳定在状态1
        {
            DownTime1 = 0;               // 等待计数归零
        }
        else if( RELOAD_KEY_2 == 0 )      // 端口输入变为低电平
        {
            KEY_STATE = 0; // 进入状态2
        }
    }
    else if( KEY_STATE == 0) // 状态2：按键按下状态
    {
        if( RELOAD_KEY_2 !=0 )             // 端口输入高电平
        {
            KEY_STATE = 1;     // 回到状态1
        }
        else if( RELOAD_KEY_2 == 0 )        // 端口输入仍旧为低电平
        {
            DownTime1++;                   // 进入计时过程
            if( DownTime1 == 2 )           // 端口低电平保持时间达到20ms
            {

                g_task_ready_flag=1;
             KEY_STATE = 1;// 按键状态强制退出状态2，改为状态1
            }
        }
    }
}
void ReloadKeyCheck_2(void)
{
    volatile static uint16_t DownTime2 = 0;        // 检测状态的时间（定时器的触发次数）
    volatile static uint8_t KEY_STATE2 = 1; // 按键的当前状态
    if( KEY_STATE2!=0 )     // 状态1：按键松开状态
    {
        if(RELOAD_KEY_1 !=0 )           // 稳定在状态1
        {
            DownTime2 = 0;               // 等待计数归零
        }
        else if( RELOAD_KEY_1 == 0 )      // 端口输入变为低电平
        {
            KEY_STATE2 = 0; // 进入状态2
        }
    }
    else if( KEY_STATE2 == 0) // 状态2：按键按下状态
    {
        if( RELOAD_KEY_1 !=0 )             // 端口输入高电平
        {
            KEY_STATE2 = 1;     // 回到状态1
        }
        else if( RELOAD_KEY_1 == 0 )        // 端口输入仍旧为低电平
        {
            DownTime2++;                   // 进入计时过程
            if( DownTime2 == 2 )           // 端口低电平保持时间达到20ms
            {

               g_systemState  =  (g_systemState  + 1)%5;

             KEY_STATE2 = 1;// 按键状态强制退出状态2，改为状态1
            }
        }
    }
}
