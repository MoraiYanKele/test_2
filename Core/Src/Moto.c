#include "Moto.h"

// typedef struct 
// {
//   float Kp;                // 比例增益
//   float Ki;                // 积分增益
//   float Kd;                // 微分增益
//   float setpoint;          // 目标值
//   float integral;          // 积分项累积值
//   float previous_error;    // 上一次误差
// } Moto_PIDController;


short GetEncoder(void)
{
  short encoder = 0;
  encoder = (short)(__HAL_TIM_GET_COUNTER(&htim1));
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  return encoder;
}

void MotoDriver(uint32_t speed)
{
  TIM2->CCR1 = speed;
  TIM2->CCR2 = 1000 - speed; 
}

float GetSpeed(short encoder)
{
  float speed;
  speed = encoder * ((3.1415 * 0.066) / (4 * 11 * 30)); 
  return speed;
}

void MotoPID_Init(Moto_PIDController* pid, float Kp, float Ki, float Kd)
{
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  // pid->setpoint = setpoint;
  pid->integral = 0;
  pid->previous_error = 0;
  pid->error = 0;
  pid->derivative = 0;
}

float Moto_LeftPID(Moto_PIDController* pid_Left, float now_LeftValue, float target_LeftValue)
{
  float output;

  pid_Left->error = target_LeftValue - now_LeftValue;
  pid_Left->integral += pid_Left->error;
  pid_Left->derivative = pid_Left->error - pid_Left->previous_error;
  output = pid_Left->Kp * pid_Left->error + pid_Left->Ki * pid_Left->integral + pid_Left->Kd * pid_Left->derivative;
  pid_Left->previous_error = pid_Left->error;
  
  return output; 
}

float Moto_RightPID(Moto_PIDController* pid_Right, float now_RightValue, float target_rightValue)
{
  float output;

  pid_Right->error = target_rightValue - now_RightValue;
  pid_Right->integral += pid_Right->error;
  pid_Right->derivative = pid_Right->error - pid_Right->previous_error;
  output = pid_Right->Kp * pid_Right->error + pid_Right->Ki * pid_Right->integral + pid_Right->Kd * pid_Right->derivative;
  pid_Right->previous_error = pid_Right->error;
  
  return output; 
}

