#ifndef __GRAYSCALE_SENSOR__
#define __GRAYSCALE_SENSOR__

#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "i2c.h"
#include "main.h" 
#include "usart.h"





#define GRAYSENSOR_ADDRESS  0x4C
#define GRAYSENSOR_READDATA 0xDD
#define PING_NETWORK_TOOL   0xAA

void Graysensor_SendOrder(uint32_t order);
void Graysensor_ReadData(uint8_t *readBuffer);
void Ping();

#endif