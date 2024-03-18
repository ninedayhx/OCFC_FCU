#ifndef __ADAPTIVE_PID_H__
#define __ADAPTIVE_PID_H__

#include "main.h"

#define FAN_PWM_MAX_VALUE   99    // 风扇PWM最大值
// PID控制器参数
#define K_P 0.8
#define K_I 0.05
#define K_D 0.1

// 定义PID参数结构体
typedef struct {
    float kp;
    float ki;
    float kd;
    float i_max;
    float i_min;
    float output_max;
    float output_min;
    float output_deadband;
    float err;
    float err_last;
    float err_sum;
    float output;
    float setpoint;
} PID_TypeDef;

// 定义风扇控制参数结构体
typedef struct {
    float temp_threshold;
    float load_threshold;
    float fan_speed;
    PID_TypeDef pid;
} FanControl_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float i_max, float i_min,
              float output_max, float output_min, float output_deadband, float setpoint);
float PID_Update(PID_TypeDef *pid, float input);
void FanControl_Update(FanControl_TypeDef *fan_control, float temp);
uint8_t calculateFanSpeed(float temperature, float power);


#endif /* __ADAPTIVE_PID_H__ */

