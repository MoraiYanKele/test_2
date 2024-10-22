/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sw_i2c.h"
#include "Moto.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
sw_i2c_interface_t i2c_interface =
{
  .sda_in = sda_in,
  .scl_out = scl_out,
  .sda_out = sda_out,
  .user_data = 0, //用户数据，可在输入输出函数里得到
};

Moto_PIDController pid_Left;
Moto_PIDController pid_Right;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */


// uint8_t graySensor_state = 0;
// uint8_t grayData;
volatile uint8_t count;
uint8_t scan_addr[128];
uint8_t gray_sensor[8];

// 串口接收数据�?????'a' openmv接收到单片机命令，正在应答；'fx' openmv在开始阶段识别的数据，x为目标房间号�?????
//             'dxx' openmv在寻找房间时识别的数据，xx为转向命令，l左转，r右转，s直走(只转向一次发x0)�????? 
uint8_t uartRxBuffer[10]; 
short encoderLeft, encoderRight;
// short encoder = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}
int fgetc(FILE *f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&huart1, &ch, 1, 0xffff);
  return ch;
}

// uint8_t graySensorReadData_Parallel()
// {
//   uint8_t ret;
//   ret |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_2) << 0; 
//   ret |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) << 1;
//   ret |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_11) << 2;
//   ret |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_4) << 3;
//   ret |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) << 4;
//   ret |= HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) << 5;
//   ret |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_8) << 6;
//   ret |= HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10) << 7;
//   HAL_Delay(1);
//   return ret; 
// }
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint8_t sizeOfBuffer = 0;
  if (huart->Instance == USART2)
  {
    sizeOfBuffer = sizeof(uartRxBuffer);
    if (sizeOfBuffer == 1 && uartRxBuffer[0] == 'a')
    {
      printf("OpenMV has answered");
    }
    else if (sizeOfBuffer >= 2)
    {
      if (uartRxBuffer[0]  == 'f')
      {
        switch (uartRxBuffer[1])
        {
          
        
          default:
            break;
        }
      }
      else if (uartRxBuffer[0] == 'd')
      {
        switch (uartRxBuffer[1])
        {
          default:
            break;
        }
      }
    }
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static int32_t timConter = 0;
  timConter++;
  if (htim->Instance == TIM2)
  {
    
    if (timConter % 10 == 0)
    {
      
      encoderRight = GetEncoder(RIGHT_ENCODER_TIM);
      encoderLeft = GetEncoder(LEFT_ENCODER_TIM);
      printf("%hd, %hd\n", encoderRight, encoderLeft);
    }
    if (timConter >= 500)
    {
      Digital_Dataget(&i2c_interface, gray_sensor);
    
      // for (int i = 0; i < 8; i++)
      // {
      //   printf("%d", gray_sensor[i]);
      // }
      printf("\n");
      timConter = 0;
    }

  } 
}



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  // sw_i2c_interface_t i2c_interface =
  // {
  //   .sda_in = sda_in,
  //   .scl_out = scl_out,
  //   .sda_out = sda_out,
  //   .user_data = 0, //用户数据，可在输入输出函数里得到
  // };
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
 
    
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  SWIIC_GPIO_Init();
  DWT_Init();	

  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  // __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

  // HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uartRxBuffer, sizeof(uartRxBuffer));
  // __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

  // MotoPID_Init(&pid_Left, 0.1, 0.1, 0.1);
  // MotoPID_Init(&pid_Right, 0.1, 0.1, 0.1);
 	

	HAL_GPIO_WritePin(RIGHT_IN_PORT, RIGHT_IN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_IN_PORT, RIGHT_IN2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEFT_IN_PORT, LEFT_IN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LEFT_IN_PORT, LEFT_IN2, GPIO_PIN_RESET);

  
  LEFT_PWM = 0;
  RIGHT_PWM = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("ready\n");
  while (1)
  {
    
    // Digital_Dataget(&i2c_interface, gray_sensor);
    
    // for (int i = 0; i < 8; i++)
    // {
    //   printf("%d", gray_sensor[i]);
    // }
    // printf("\n");
    // HAL_Delay(500);
    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
