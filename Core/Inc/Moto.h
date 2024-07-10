#ifndef __MOTO_H__
#define __MOTO_H__

#include "main.h"
#include "tim.h"

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct 
{
    float Kp;                // 比例增益
    float Ki;                // 积分增益
    float Kd;                // 微分增益
    // float setpoint;          // 目标值
    float integral;          // 积分项累积值
    float derivative;        // 当前微分项
    float error;             // 当前误差
    float previous_error;    // 上一次误差
    
} Moto_PIDController;

short GetEncoder(void);
void MotoDriver(uint32_t speed);
float GetSpeed(short encoder);
void MotoPID_Init(Moto_PIDController* pid, float Kp, float Ki, float Kd);
float Moto_LeftPID(Moto_PIDController* pid_Left, float now_LeftValue, float target_LeftValue);
float Moto_RightPID(Moto_PIDController* pid_Right, float now_RightValue, float target_rightValue);

#endif

