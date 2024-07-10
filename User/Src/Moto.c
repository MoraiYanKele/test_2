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
    htim3->
}
