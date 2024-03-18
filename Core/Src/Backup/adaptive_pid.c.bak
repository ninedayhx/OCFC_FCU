/*
 * adaptive_pid.c
 *
 *  Created on: May 7, 2023
 *      Author: edgar
 */

#include "adaptive_pid.h"


extern SysControl_TypeDef sysControl;

//
/**
 * 初始化PID参数
 * kp：比例常数
 * ki：积分常数
 * kd：微分常数
 * i_max：积分项上限
 * i_min：积分项下限
 * output_max：输出上限
 * output_min：输出下限
 * output_deadband：输出死区（当误差小于该值时，输出为0）
 * setpoint：期望值（也就是您希望控制系统达到的目标值）
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float i_max, float i_min,
              float output_max, float output_min, float output_deadband, float setpoint) {
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_max = i_max;
    pid->i_min = i_min;
    pid->output_max = output_max;
    pid->output_min = output_min;
    pid->output_deadband = output_deadband;
    pid->err = 0.0f;
    pid->err_last = 0.0f;
    pid->err_sum = 0.0f;
    pid->output = 0.0f;
    pid->setpoint = setpoint;
}

// 自适应PID控制函数
float PID_Update(PID_TypeDef *pid, float input) {
    pid->err = pid->setpoint - input;
    pid->err_sum += pid->err;
    if (pid->err_sum > pid->i_max) {
        pid->err_sum = pid->i_max;
    } else if (pid->err_sum < pid->i_min) {
        pid->err_sum = pid->i_min;
    }
    pid->output = pid->kp * pid->err + pid->ki * pid->err_sum + pid->kd * (pid->err - pid->err_last);
    if (pid->output > pid->output_max) {
        pid->output = pid->output_max;
    } else if (pid->output < pid->output_min) {
        pid->output = pid->output_min;
    }
    if (fabs(pid->err) < pid->output_deadband) {
        pid->output = 0.0f;
    }
    pid->err_last = pid->err;
    return pid->output;
}

// 风扇控制函数
void FanControl_Update(FanControl_TypeDef *fan_control, float temp) {
    float fan_speed = fan_control->fan_speed;
    fan_speed -= PID_Update(&(fan_control->pid), temp);

    // 对风扇转速进行限制
    if (fan_speed > 99.0f) {
        fan_speed = 99.0f;
    } else if (fan_speed < 0.0f) {
        fan_speed = 0.0f;
    }
    fan_control->fan_speed = fan_speed;
    // 控制风扇转速
    sysControl.Expected_FC_Fan_Speed = fan_speed;
}

//转速映射
uint8_t powerMap(float value, float inMin, float inMax, uint8_t outMin, uint8_t outMax)
{
  return (value - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
}

//风扇控制
uint8_t calculateFanSpeed(float temperature, float power)
{
  if (temperature < FC_NORMAL_TEMP)
  {
    // 映射功率范围0-1.3到风扇转速0-99
    uint8_t fanSpeed = powerMap(power, 0, SYS_MAX_POWER, 35, 99);

    return fanSpeed;
  }
  else if(temperature > FC_NORMAL_TEMP)
  {
    // 温度高于FC_NORMAL_TEMP时，输出99风扇转速
    return 99;
  }
}

