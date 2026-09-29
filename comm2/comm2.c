#include "comm2.h"

// 独立接收缓存，和原来comm缓存区分开
 volatile uint8_t use_gyro_pid = 0;      // 0=灰度巡线PID  1=陀螺仪角度PID
uint8_t rx2_buffer[RX_BUFFER_MAX_LEN] = {0};
uint16_t rx2_idx = 0;

// 灰度传感器数据相关全局
uint8_t sensor_status[SENSOR_CH_NUM] = {0};
bool track_data_ready = false;
uint16_t is_notonblakline = 0;
volatile uint8_t line_exist=0;
//////////////
volatile uint8_t stable_line_flag = 0;    // 防抖之后稳定标志
uint16_t stable_filter_cnt = 0;
/**
 * @brief 串口初始化，使能外设+接收中断，上电发送灰度启动指令 $0,0,1#
 */
void comm2_uart_init(void)
{
    DL_UART_enable(UART_COMM2_INST);

    NVIC_ClearPendingIRQ(UART_COMM2_IRQN);
    NVIC_EnableIRQ(UART_COMM2_IRQN);

    // 发送启动串
    comm2_send_start_cmd();
}
// void comm2_uart_init(void)
// {
//     DL_UART_enable(UART_COMM2_INST);

//     NVIC_ClearPendingIRQ(UART_COMM2_IRQN);
//     NVIC_EnableIRQ(UART_COMM2_IRQN);

//     // 初始化阶段关闭接收中断，杜绝上电杂波触发中断挂起
//     //DL_UART_disableInterrupt(UART_COMM2_INST, DL_UART_INTERRUPT_RX);

//     // 溢出宏不存在，直接删掉溢出清除，只保留基础配置
//     comm2_send_start_cmd();
// }
/**
 * @brief 发送启动指令 "$0,0,1#"，沿用你项目标准阻塞发送逻辑
 */
void comm2_send_start_cmd(void)
{
    uint8_t start_cmd[] = "$0,0,1#";
    uint8_t cmd_len = sizeof(start_cmd) - 1;

    for(uint8_t i = 0; i < cmd_len; i++)
    {
        while(DL_UART_isBusy(UART_COMM2_INST));
        DL_UART_Main_transmitData(UART_COMM2_INST, start_cmd[i]);
    }

    // 200ms裸机延时

}

/**
 * @brief 解析$D开头、#结尾灰度报文，完全复刻你STM32逻辑
 */
void Parse_Sensor_Data(void)
{
    if(rx2_idx >= 42 && rx2_buffer[0] == '$' && rx2_buffer[1] == 'D')
    {
        for(int i = 0; i < SENSOR_CH_NUM; i++)
        {
            int index = 6 + (i * 5);
            if(rx2_buffer[index] == '0' || rx2_buffer[index] == '1')
            {
                sensor_status[i] = rx2_buffer[index] - '0';
            }
            else return;
        }
        track_data_ready = true;
    }
}

/**
 * @brief 加权计算循迹偏差，逻辑原样移植
 * @return int8_t 负数左偏，正数右偏，0无黑线
 */
int8_t Get_Track_Error(void)
{ //se+=1;
    // static const int8_t sensor_weights[SENSOR_CH_NUM] = {-20, -7, -4, -1, 1, 4, 7, 20};
    // 8路从左到右：左最外侧 → 右最外侧，严格左右对称
static const int16_t sensor_weights[SENSOR_CH_NUM] = {-6, -4, -3, -1, 1, 3, 4, 6};
    //static int8_t corner_detect_cnt = 0;//多次判断是否检测为直角
    int16_t weighted_sum = 0;
    //uint8_t active_count = 0;
      uint8_t line_sensor_on_cnt = 8;
    for(int i = 0; i < SENSOR_CH_NUM; i++)
    {
        if(sensor_status[i] == 1)
        {
            weighted_sum += sensor_weights[i];
               line_sensor_on_cnt --;
        }
    }


    int8_t error = (int8_t)weighted_sum;
    gray_error=-error;
        // line_exist=line_sensor_on_cnt;

//   if(  line_sensor_on_cnt!=0&& g_task_ready_flag != 0)
//     {
//         corner_detect_cnt++;
//        // corner_detectcnt=0;
//     }
//     else
//     {
//         // 脱离弯道，计数清零，防止单次抖动累加
//         //corner_detectcnt++;
//         corner_detect_cnt = 0;

//     }

//     if(corner_detect_cnt >= CORNER_TRIG_CNT)
//     {

//         line_exist=1;
//     }
// ========== 以 line_exist 作为状态变量的switch状态机 ==========
switch( line_exist )
{
    case 0:
    {
  if(  line_sensor_on_cnt!=0&& g_task_ready_flag != 0)
    {
        corner_detect_cnt++;
       // corner_detectcnt=0;
    }
    else
    {
        // 脱离弯道，计数清零，防止单次抖动累加
        //corner_detectcnt++;
        corner_detect_cnt = 0;

    }

    if(corner_detect_cnt >= CORNER_TRIG_CNT)
    {

        line_exist=1;
        corner_detect_cnt=0;
    }
        break;
    }
    case 1:
    {
  if(  line_sensor_on_cnt==0&& g_task_ready_flag != 0)
    {
        corner_detect_cnt--;
       // corner_detectcnt=0;
    }
    else
    {
        // 脱离弯道，计数清零，防止单次抖动累加
        //corner_detectcnt++;
        corner_detect_cnt = 0;

    }

    if(corner_detect_cnt <=-1600)
    {

        line_exist=0;
        corner_detect_cnt=0;
    }
        break;
    }
    default:
    {
        // 异常兜底，复位状态

        break;
    }
}


    return -error;
}

/**
 * @brief UART1中断服务函数（注意：只能存在一份IRQHandler！）
 * 重要：如果你原来comm.c里已经写了 void UART_1_INST_IRQHandler，不能两份同时定义
 * 解决办法看下方说明
 */
void UART_1_INST_IRQHandler(void)
{
    uint32_t int_flag = DL_UART_getPendingInterrupt(UART_COMM2_INST);
    if(int_flag != DL_UART_IIDX_RX)
        return;
    //se+=1;
    uint8_t rx_byte = DL_UART_Main_receiveData(UART_COMM2_INST);
    if (rx2_idx >= RX_BUFFER_MAX_LEN - 1) {
        rx2_idx = 0;
    }
    rx2_buffer[rx2_idx++] = rx_byte;

    if(rx_byte == '#')
    {
        rx2_buffer[rx2_idx] = '\0';
        Parse_Sensor_Data();
        rx2_idx = 0;
    }
    Get_Track_Error();
}
// void UART_COMM2_IRQHandler(void)
// {
//     if(DL_UART_getInterruptStatus(UART_COMM2_INST, DL_UART_INTERRUPT_RX))
//     {
//         uint8_t recv_data = DL_UART_receiveData(UART_COMM2_INST);

//         // 上电未解锁时，直接丢弃垃圾数据，不执行业务解析，避免中断积压
//         if(uart_rx_allow == 0)
//         {
//             return;
//         }

//         // ========== 下方写你原本正常的串口数据解析代码 ==========


//     uint32_t int_flag = DL_UART_getPendingInterrupt(UART_COMM2_INST);
//     if(int_flag != DL_UART_IIDX_RX)
//         return;

//     uint8_t rx_byte = DL_UART_Main_receiveData(UART_COMM2_INST);
//     rx2_buffer[rx2_idx++] = rx_byte;

//     if(rx_byte == '#' || rx2_idx >= RX_BUFFER_MAX_LEN)
//     {
//         rx2_buffer[rx2_idx] = '\0';
//         Parse_Sensor_Data();
//         rx2_idx = 0;
//     }


//     }
// }
float_t Get_Target_Yaw(uint8_t state, float_t yaw)
{
    switch(state)
    {
        case STATE_TASK1:
            return yaw;
        case STATE_TASK2:
        if( cross_point_cnt==2)
            // return -175.0;
         return yaw-15;
        else
            return yaw;
        case STATE_TASK3:
        if (g_task_ready_flag) {


                  switch(cross_point_cnt)
            {
                case 0:
                    return yaw ;

                case 2:
                    return yaw + 55.0f;

                default:
                    // 超出预设计数，保持当前角度
                    return yaw;
            }
        }
        case STATE_TASK4:
        if (g_task_ready_flag) {


                  switch(cross_point_cnt)
            {
                case 0:
                    return yaw - 38.0;

                case 2:
                    return yaw + 52.7f;

                case 4:
                    return yaw - 53.2f;

                case 6:
                    return yaw + 52.7f;

                case 8:
                    return yaw -53.2f;

                case 10:
                    return yaw + 52.7f;

                case 12:
                    return yaw -53.2f;

                case 14:
                    return yaw +52.7f;
                default:
                    // 超出预设计数，保持当前角度
                    return yaw;
            }
        }
        default:
            // 空闲/未知任务，直接返回原始yaw，避免失控
            return yaw;
    }
}
