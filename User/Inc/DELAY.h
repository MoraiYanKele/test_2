#ifndef __DELAY_H
#define __DELAY_H 			   

#include "stm32g4xx.h"

#define delay_ms HAL_Delay

void delay_us(uint32_t udelay);
void DWT_Init(void);
void DWT_Delay_us(uint32_t us); // microseconds

#endif





























