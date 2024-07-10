#include "Moto.h"

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