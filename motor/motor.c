#include "motor.h"
#include "main.h"
#include "ti_msp_dl_config.h"

/**
 * @brief 电机初始化
 */
void Motor_Init(void)
{
    // SysConfig 默认比较值为 500；在启动计数器前先归零，避免上电瞬间输出。
    Motor_Set_PWM(0, 0);
    DL_Timer_startCounter(PWM_0_INST);
}

/**
 * @brief 左电机控制 (PWMA: PA8, AIN2: PB6, AIN1: PB8)
 */
void Motor_Left_Set(int32_t pwm)
{
    if (pwm >  MOTOR_MAX_PWM) pwm =  MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;

    if (pwm >= 0) // 正转
    {
        // AIN1=1, AIN2=0 (PB8=1, PB6=0)
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_8);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_6);
        // 修正：使用通用 Timer 设置函数
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)pwm, DL_TIMER_CC_0_INDEX);
    }
    else // 反转
    {
        // AIN1=0, AIN2=1 (PB8=0, PB6=1)
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_8);
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_6);
        // 修正：使用通用 Timer 设置函数
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (uint32_t)(-pwm), DL_TIMER_CC_0_INDEX);
    }
}

/**
 * @brief 右电机控制 (PWMB: PA7, BIN1: PB23, BIN2: PB27)
 */
void Motor_Right_Set(int32_t pwm)
{
    if (pwm >  MOTOR_MAX_PWM) pwm =  MOTOR_MAX_PWM;
    if (pwm < -MOTOR_MAX_PWM) pwm = -MOTOR_MAX_PWM;

    if (pwm >= 0) // 正转
    {
        // BIN1=1, BIN2=0 (PB23=1, PB27=0)
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_23);
        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_27);
        // 修正：使用通用 Timer 设置函数
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (int32_t)pwm, DL_TIMER_CC_1_INDEX);
    }
    else // 反转
    {
        // BIN1=0, BIN2=1 (PB23=0, PB27=1)

        DL_GPIO_clearPins(GPIOB, DL_GPIO_PIN_23);
        DL_GPIO_setPins(GPIOB, DL_GPIO_PIN_27);
        // 修正：使用通用 Timer 设置函数
        DL_Timer_setCaptureCompareValue(PWM_0_INST, (int32_t)(-pwm), DL_TIMER_CC_1_INDEX);
    }
}

/**
 * @brief 综合设置左右电机速度
 */
void Motor_Set_PWM(int32_t left_pwm, int32_t right_pwm)
{
    Motor_Left_Set(left_pwm);
    Motor_Right_Set(right_pwm);
}
/**
 * @brief 任务启动初始化函数
 * @param left_start_pwm  左电机初始占空比
 * @param right_start_pwm 右电机初始占空比
 * @note  该函数内部带有静态标志位，确保在一次任务运行期间仅执行一次
 */
void Motor_Task_Start_Init(int32_t left_start_pwm, int32_t right_start_pwm) {
    static uint8_t has_initialized = 0; // 静态变量，用于记录是否已执行

    // 如果任务已就绪且尚未初始化
    if (g_task_ready_flag != 0 && has_initialized == 0) {
        Motor_Set_PWM(left_start_pwm, right_start_pwm);
        has_initialized = 1; // 锁定，防止重复执行
    }

    // 如果任务被停止（确定键复位），则重置初始化锁，为下一次启动做准备
    if (g_task_ready_flag == 0) {
        has_initialized = 0;
    }
}
/**
 * @brief 紧急停止并触发蜂鸣器报警
 * @note  严格按照要求：置零标志位 -> 开启蜂鸣器 -> 延迟 -> 关闭蜂鸣器
 */
// void Buzzer_TaskFinish(void) {
//     if(g_trace_error_count>20){
//     // 1. 确定键（任务启动标志）置为 0
//     g_task_ready_flag = 0;

//     // 2. 左右电机立即停止（防止标志位清零后的惯性）
//     Motor_Set_PWM(0, 0);

//     // 3. 响起蜂鸣器
//     DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_9);

//     // 4. 延迟一段时间 (例如 500ms)
//     mspm0_delay_ms(500);

//     // 5. 关闭蜂鸣器
//     DL_GPIO_setPins(GPIOA, DL_GPIO_PIN_9);
//     }
// }
