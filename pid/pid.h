#ifndef __PID_H
#define __PID_H
 #define ANGLE_ARRIVE_THRESHOLD 5
typedef struct {
    float p;
    float i;
    float d;

    float target;
    float now;

    float pout;
    float iout;
    float dout;
    float out;

    float error[3];
} pid_t;

void pid_init(pid_t *pid, float p, float i, float d);
void pid_cal(pid_t *pid);
void pid_cal_trace(pid_t *pid);
void pidout_limit(pid_t *pid);
float Yaw_error_zzk(float Target, float Now);
void Control_Execute_Angle(void);
void Control_Execute_Trace(void);

#endif