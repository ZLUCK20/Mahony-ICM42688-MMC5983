#ifndef ICM42688_H
#define ICM42688_H

#include "stm32f4xx_hal.h"   
#include "spi.h"
#include "gpio.h"
#include <stdint.h>

/* CS 引脚定义 */
#define GYRO_CS1_HIGH()  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_SET)
#define GYRO_CS1_LOW()   HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, GPIO_PIN_RESET)

/* 基础 SPI 读写 */
void Gyro_WriteReg(uint8_t reg, uint8_t value);
void Gyro_ReadReg(uint8_t reg, uint8_t *rx_data, uint8_t len);

/* 陀螺仪设备功能 */
uint8_t Icm_ReadWhoAmI(void);
void    Icm_SwitchBank(uint8_t bank);
void    Icm_Init(void);
void    Icm_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az);
void    Icm_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz);

#endif /* __ICM42688_H__ */
