#ifndef __MOTO_H__
#define __MOTO_H__

#include "main.h"
#include "tim.h"

#include "stm32f1xx_hal.h"
#include <stdint.h>


short GetEncoder(void);
void MotoDriver(uint32_t speed);
float GetSpeed(short encoder);


#endif