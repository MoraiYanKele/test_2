/*
 * 通用软件IIC使用方式
 * 首先需要定义三个函数
 * 
 * 根据bit=0 或者bit=1 在SDA阵脚输出对应的电平，然后根据自己需要的IIC频率delay N us
 * void sda_out(uint8_t bit, void *user_data)
 *
 * 读取SDA电平值然后返回低电平0 或者是高电平1，然后根据自己需要的IIC频率delay N us
 * uint8_t sda_in(void *user_data)
 *
 * 根据bit=0 或者bit=1 在SCL阵脚输出对应的电平，然后根据自己需要的IIC频率delay N us
 * void scl_out(uint8_t bit, void *user_data)
 */

#ifndef __SW_I2C_H__
#define __SW_I2C_H__

#include <stdint.h>
#include "gw_grayscale_sensor.h"
#include "stm32g4xx.h"
#include "DELAY.h"

//软件IIC引脚，修改软件IIC只需修改这一块
#define SW_I2C1_SCL_PORT GPIOA
#define SW_I2C1_SDA_PORT GPIOA
#define SW_I2C1_PIN_SCL GPIO_PIN_2
#define SW_I2C1_PIN_SDA GPIO_PIN_3
#define SW_I2C1_SCL_RCC_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define SW_I2C1_SDA_RCC_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()

#define ACK 0x0 // acknowledge (SDA LOW)
#define NACK 0x1 // not acknowledge (SDA HIGH)
#define LOW 0x0
#define HIGH 0x1
#define I2C_READ 0x1
#define I2C_WRITE 0x0

/* 模拟模式数据改成0, 开关模式数据改成1 */
#define GW_READ_DIGITAL_DATA 1

typedef struct {
	void (*sda_out)(uint8_t BIT, void *user_data);
	uint8_t (*sda_in)(void *user_data);
	void (*scl_out)(uint8_t BIT, void *user_data);
	void *user_data;
} sw_i2c_interface_t;

//软件IIC GPIO初始化
void SWIIC_GPIO_Init(void);
/**
 * @brief 从IIC总线上的设备读取多个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节)
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data, uint8_t data_length);
int8_t sw_i2c_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t *data, uint8_t data_length);

/**
 * @brief 从IIC总线上的设备读取一个字节
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param[out] data 读取到的字节
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_read_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t *data);
int8_t sw_i2c_write_byte(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, const uint8_t data);

/**
 * @brief 读取IIC总线从设备的寄存器数据. 即先写入寄存器地址,无终止位,再连续读取所需数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 读取到的字节数组
 * @param data_length 读取大小(字节),不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_read(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, uint8_t *data, uint8_t data_length);

/**
 * @brief 写入IIC总线从设备的寄存器. 即先写入寄存器地址,再连续写入数组中的数据
 * @param i2c_interface
 * @param dev_addr 从设备地址
 * @param mem_addr 寄存器地址
 * @param[out] data 连续写入的数据
 * @param data_length 所写入的字节大小,不包括寄存器地址本身
 * @return 0:成功, 1:错误
 */
int8_t sw_i2c_mem_write(sw_i2c_interface_t *i2c_interface, uint8_t dev_addr, uint8_t mem_addr, const uint8_t *data, uint8_t data_length);

/* 定义sda输出函数 bit=0为低电平 bit=1为高电平 */
void sda_out(uint8_t BIT, void *user_data);
/* 定义sda读取函数 bit 为返回的电平值 */
uint8_t sda_in(void *user_data);
/* 定义scl时钟输出函数 bit=0为低电平 bit=1为高电平 */
void scl_out(uint8_t BIT, void *user_data);
/**
 * i2c地址扫描
 * @param scan_addr 扫描出来的地址存放,数值不为0的为扫描到的地址，扫到的地址会挨个放在数组的最前面
 * @return 返回扫描到的设备数量, 0为无设备发现
 */
uint8_t i2c_scan(sw_i2c_interface_t *i2c_interface, uint8_t *scan_addr);
//数字量数据获取，存放到gray_sensor中
void Digital_Dataget(sw_i2c_interface_t *i2c_interface, uint8_t *gray_sensor);
//模拟量数据获取，存放到gray_sensor中
void Analog_Dataget(sw_i2c_interface_t *i2c_interface, uint8_t *gray_sensor);
//根据GW_READ_DIGITAL_DATA确定读取方式, GW_READ_DIGITAL_DATA = 0 模拟数据模式、GW_READ_DIGITAL_DATA = 1 开关数据模式
void DataGet(sw_i2c_interface_t *i2c_interface, uint8_t *gray_sensor);

#endif //SW_I2C_H_GUARD
