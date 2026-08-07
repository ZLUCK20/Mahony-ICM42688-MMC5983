/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "oled.h"
#include "bmp.h"
#include "mahony_filter.h"
#include "NRF24L01.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern int tim; //定时中断测试代码
extern float isr_time_us;       // ISR 耗时 (微秒) /*四元数mahony算法运行时间测试变量*/
extern uint32_t isr_max_cycles; // ISR 最大周期数  /*四元数mahony算法运行时间测试变量*/

uint8_t usart3_rx_byte;  // 串口三接收到的字符
extern volatile uint8_t usart3_cmd_flag;  // 串口三命令标志

char nrf_buf[32];        // OLED 显示用缓冲区

float pitch, roll, yaw;  // Mahony 解算输出（弧度，主循环内经 Get_Angle 转角度）
// int32_t mx_i, my_i, mz_i;//磁力计原始adc值的变量
// extern Vector3f omega_corrected;//纠正后的omega
volatile uint8_t key_flag[4] = {0};  // [0]不用, [1]=PC0, [2]=PC1, [3]=PC2
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_TIM5_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  OLED_Clear();

  // 冷启动时 ICM42688 需要足够时间完成上电初始化
  HAL_Delay(600);
  IMU_Init();                              // 初始化 ICM42688 + MMC5983

  HAL_TIM_Base_Start_IT(&htim5);   // start TIM5 1kHz Mahony interrupt
  HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);  // 蓝牙串口三启动单字节中断接收
  NRF24L01_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  
      /*陀螺仪ICM42688*/
    //串口打印原始数据
    // Print_initialvalue();
    //读取磁力计原始数据无校准归一化
    // Mag_ReadData(&mx_i, &my_i, &mz_i);
    // printf("Mag: %d %d %d\r\n", mx_i, my_i, mz_i);

  // //printf("\n\naccel_norm: accelx: %.3f,accely: %.3f,accelz: %.3f",f_ax,f_ay,f_az);
  // //printf("\n\nmag_norm: magx: %.3f,magy: %.3f,magz: %.3f",f_mx,f_my,f_mz);
  // //printf("%.3f, %.3f, %.3f\n", omega_corrected.x, omega_corrected.y, omega_corrected.z);

     //printf("ISR: %.1f us, max: %lu cycles\n", isr_time_us, isr_max_cycles);/*四元数mahony算法运行时间测试语句*/
  

  /*打印mahony滤波融合以后的角度值*/
  // 先快照再转换，避免显示/打印过程中被 1kHz 中断刷新
  float p = pitch, r = roll, y = yaw;
  Get_Angle(&p, &r, &y);
  printf("%.3f, %.3f, %.3f\n", p, r, y);

  // OLED 显示 Mahony 解算角度（p/r/y 为角度值）
  sprintf(nrf_buf, "pitch:%6.1f", p);
  OLED_ShowString(L_X1, H_Y2, (uint8_t *)nrf_buf, 16, 1);
  sprintf(nrf_buf, "roll:%6.1f", r);
  OLED_ShowString(L_X1, H_Y3, (uint8_t *)nrf_buf, 16, 1);
  sprintf(nrf_buf, "yaw:%6.1f", y);
  OLED_ShowString(L_X1, H_Y4, (uint8_t *)nrf_buf, 16, 1);

  // 运行时间显示（右上角，沿用原布局）
  char time_buf[16];
  sprintf(time_buf, "time:%02ds", (int)(tim / 1000));
  OLED_ShowString(L_X5, H_Y1, (uint8_t *)time_buf, 16, 1);

  OLED_Refresh();


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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)//按键中断
{
    /* 只设标志，不延迟不printf */
    if     (GPIO_Pin == GPIO_PIN_0) key_flag[1] = 1;
    else if(GPIO_Pin == GPIO_PIN_1) key_flag[2] = 1;
    else if(GPIO_Pin == GPIO_PIN_2) key_flag[3] = 1;
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        HAL_UART_Receive_IT(&huart3, &usart3_rx_byte, 1);  // 重新使能单字节接收
    }
}
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
#ifdef USE_FULL_ASSERT
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
