#include "pid.h"
#include <math.h>
#include "main.h"
// 角度环 PID 计算（HWT101 航向角）
void pid_cal(pid_t *pid)
{
    // 计算 360° 角度误差
    pid->error[0] = Yaw_error_zzk(pid->target, pid->now);

    // 位置式 PID
    pid->pout = pid->p * pid->error[0];
    pid->iout += pid->i * pid->error[0];
    pid->dout = pid->d * (pid->error[0] - pid->error[1]);

    // 积分限幅，防止超调
    if(pid->iout >80)  pid->iout = 80;
    if(pid->iout < -80) pid->iout = -80;

    // 总输出
    pid->out = pid->pout + pid->iout + pid->dout;

    // 保存历史偏差
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

// 巡线 PID 计算（灰度传感器误差）
void pid_cal_trace(pid_t *pid)
{
    // 普通误差计算

    //pid->now= Get_Track_Error();

    pid->error[0] = pid->target - pid->now;

    // 位置式 PID
    pid->pout = pid->p * pid->error[0];
    pid->iout += pid->i * pid->error[0];
    pid->dout = pid->d * (pid->error[0] - pid->error[1]);

    // 巡线积分限幅更小
    if(pid->iout > 15)  pid->iout = 15;
    if(pid->iout < -15) pid->iout = -15;

    pid->out = pid->pout + pid->iout + pid->dout;

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
}

// PID 初始化
void pid_init(pid_t *pid, float p, float i, float d)
{
    pid->p = p;
    pid->i = i;
    pid->d = d;

    pid->now = 0;
    pid->target = 0;
    pid->out = 0;

    pid->pout = 0;
    pid->iout = 0;
    pid->dout = 0;

    pid->error[0] = 0;
    pid->error[1] = 0;
    pid->error[2] = 0;
}

// PID 输出限幅
void pidout_limit(pid_t *pid)
{
    if(pid->out >= 400)
        pid->out = 400;
    if(pid->out <= -400)
        pid->out = -400;
}

// 360° 角度误差计算（MPU 专用）
float Yaw_error_zzk(float Target, float Now)
{
    static float error;
    if (Target > 0)
    {
        if (Now <= 0)
        {
            if (fabs(Now) < (180 - Target))
                error = fabs(Now) + Target;
            else
                error = -(180 - Target) - (180 - fabs(Now));
        }
        else
            error = Target - Now;
    }
    else if (Target < 0)
    {
        if (Now > 0)
        {
            if (Now > Target + 180)
                error = (180 - Now) + (180 - fabs(Target));
            else if (Now < Target + 180)
                error = -(fabs(Target) + Now);
        }
        else if (Now < 0)
            error = -(fabs(Target) - fabs(Now));
    }
    else
        error = Target - Now;

    return error;
}

 /*
 * @brief 巡线控制执行函数 (对应 STATE_TASK1)
 * @note  基于 trace_pid 计算差速，叠加在基础速度上
 */
void Control_Execute_Trace(void) {
    // 1. 计算 PID 输出
    // 注意：trace_pid.now 已在 main 的数据处理部分更新为 red_line_deviation
    int32_t left_speed_Trace = 210;
    int32_t right_speed_Trace = 210;

    trace_pid.now= gray_error;
    pid_cal_trace(&trace_pid);

    // 2. 输出限幅
    pidout_limit(&trace_pid);

    left_speed_Trace  =  left_speed_Trace - trace_pid.out;
    right_speed_Trace = right_speed_Trace + trace_pid.out;

    //static int8_t left_speed =BASE_SPEED + (int32_t)trace_pid.out
    //static int8_t right_speed =BASE_SPEED - (int32_t)trace_pid.out
    // 3. 作用于电机：基础速度 + 差速
    Motor_Set_PWM(left_speed_Trace,right_speed_Trace);


}

/**
 * @brief 角度控制执行函数 (对应 STATE_TASK2, 3, 4)
 * @note  基于 angle (HWT101) 计算输出，实现原地转向或航向保持
 */
void Control_Execute_Angle(void) {
    // 1. 声明静态变量：只在第一次进入函数时初始化为 BASE_SPEED
    // 之后每次调用该函数，它们都会保留上一次运行结束时的数值
     int16_t left_speed_angle = 180;
     int16_t right_speed_angle = 180;

     angle.now = yaw;
   if(angle_init_flag) {

   // angle.target = Get_Target_Yaw(uint8_t state, yaw);
    angle.target = Get_Target_Yaw(g_systemState, yaw);
    angle_init_flag = 0;
}
// else {
//     // float angle_err = fabsf(angle.target - yaw);
//     float angle_err = fabsf(Yaw_error_zzk(angle.target, yaw));
//     if(angle_err < ANGLE_ARRIVE_THRESHOLD)
//     {
//         use_gyro_pid = 0;
//         angle_init_flag =1;
//         //cycle_flag=0;
//     }
// }
    pid_cal(&angle);
    pidout_limit(&angle);

    // 3. 将 PID 输出转换为 16 位整数
    int16_t rotation_adjust = (int16_t)angle.out;

    // 4. “继承”并更新速度
    // 注意：这里的逻辑取决于你希望如何“继承”。
    // 如果你希望在“上一次的速度”基础上累加偏移，用 += ；
    // 如果你希望在“基础速度”上叠加偏移，用 = 。
    // left_speed_angle  = left_speed_angle - rotation_adjust;
    // right_speed_angle = right_speed_angle + rotation_adjust;
   left_speed_angle  = left_speed_angle - rotation_adjust;
    right_speed_angle = right_speed_angle + rotation_adjust;
    // 5. 限幅保护：防止 int16_t 溢出或超过电机负载
    if(left_speed_angle > 995)  left_speed_angle = 995;
    if(left_speed_angle < -995) left_speed_angle = -995;
    // ... 对 right_speed 执行同样操作
    if(right_speed_angle > 995)  right_speed_angle = 995;
    if(right_speed_angle < -995) right_speed_angle = -995;
    // 6. 作用于电机
    Motor_Set_PWM(left_speed_angle, right_speed_angle);
}

