#ifndef __MOTO_H__
#define __MOTO_H__

#include "main.h"
#include "tim.h"
#include "stm32f1xx_hal.h"
#include <stdint.h>

#define LEFT_IN_PORT GPIOB
#define LEFT_IN1 GPIO_PIN_12
#define LEFT_IN2 GPIO_PIN_13

#define RIGHT_IN_PORT GPIOE
#define RIGHT_IN1 GPIO_PIN_2
#define RIGHT_IN2 GPIO_PIN_3

#define LEFT_PWM TIM3->CCR1
#define RIGHT_PWM TIM3->CCR2

#define LEFT_ENCODER_TIM &htim4
#define RIGHT_ENCODER_TIM &htim1


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

short GetEncoder(TIM_HandleTypeDef* htim);
void MotoDriver(uint32_t speed);
float GetSpeed(short encoder);
void MotoPID_Init(Moto_PIDController* pid, float Kp, float Ki, float Kd);
float Moto_LeftPID(Moto_PIDController* pid_Left, float now_LeftValue, float target_LeftValue);
float Moto_RightPID(Moto_PIDController* pid_Right, float now_RightValue, float target_rightValue);

#endif

