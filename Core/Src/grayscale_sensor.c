#include "grayscale_sensor.h"
#include <stdio.h>
#include <string.h>






void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    printf("进入i2c发送中断回调函数");
    if(hi2c == &hi2c1 && graySensor_state == 2)
    {
        
        printf("进入i2c发送中断回调函数");
       
        graySensor_state = 3;
    }
  
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    printf("%d",grayData);
}





void Graysensor_SendOrder(uint32_t order)
{
    // printf("in,Graysensor_SendOrder");
    uint8_t sendBuffer = order;
    HAL_I2C_Master_Transmit_DMA(&hi2c1, GRAYSENSOR_ADDRESS, &sendBuffer, 1);
    printf("in,Graysensor_SendOrder");
}

void Graysensor_ReadData(uint8_t *readBuffer)
{
    
    HAL_I2C_Master_Receive_DMA(&hi2c1, GRAYSENSOR_ADDRESS, readBuffer, 1);
    
}

void Ping()
{
    printf("in,ping");
    uint8_t sensor_status = 0;
    uint8_t seed = 0xAA;
    while (sensor_status != 0x66)
    {
        HAL_I2C_Master_Transmit(&hi2c1, GRAYSENSOR_ADDRESS, &seed, 1, HAL_MAX_DELAY);
        
        Graysensor_ReadData(&sensor_status);
        printf("ping %d\n", sensor_status);

    }
    printf("out, ping");

}