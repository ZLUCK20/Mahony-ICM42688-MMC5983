#ifndef MMC5983_H
#define MMC5983_H

#include "stm32f4xx_hal.h"
#include "spi.h"
#include "gpio.h"
#include <stdint.h>

/* 磁力计片选宏定义 */
#define MAG_CS2_HIGH()  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_SET)
#define MAG_CS2_LOW()   HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, GPIO_PIN_RESET)

/* MMC5983 寄存器地址 (数据手册 Rev. A Page 13) */
#define MMC5983_XOUT0       0x00    // Xout[17:10] 高字节
#define MMC5983_XOUT1       0x01    // Xout[9:2]   中字节
#define MMC5983_YOUT0       0x02    // Yout[17:10] 高字节
#define MMC5983_YOUT1       0x03    // Yout[9:2]   中字节
#define MMC5983_ZOUT0       0x04    // Zout[17:10] 高字节
#define MMC5983_ZOUT1       0x05    // Zout[9:2]   中字节
#define MMC5983_XYZOUT2     0x06    // [7:6]=X[1:0], [5:4]=Y[1:0], [3:2]=Z[1:0]
#define MMC5983_TOUT        0x07    // 温度输出
#define MMC5983_STATUS      0x08    // 状态寄存器
#define MMC5983_CTRL0       0x09    // Internal Control 0
#define MMC5983_CTRL1       0x0A    // Internal Control 1 (含 SW_RST bit7)
#define MMC5983_CTRL2       0x0B    // Internal Control 2 (Cmm_en, CM_Freq, Prd_set)
#define MMC5983_CTRL3       0x0C    // Internal Control 3 (SPI 3线模式等)
#define MMC5983_WHOAMI      0x2F    // Product ID 1
#define MMC5983_CHIP_ID     0x30    // 期望的 WHO_AM_I 值

/* STATUS 寄存器位定义 (bit 0: Meas_M_Done, bit 1: Meas_T_Done) */
#define MMC5983_STATUS_MEAS_M_DONE  0x01    // 磁场测量完成标志
#define MMC5983_STATUS_MEAS_T_DONE  0x02    // 温度测量完成标志

/* CTRL1 位定义 */
#define MMC5983_CTRL1_SW_RST     0x80    // bit7: 软件复位

/* 18 位数据范围 (数据手册 Page 2) */
#define MMC5983_18BIT_MAX     262143   // 2^18 - 1
#define MMC5983_18BIT_ZERO    131072   // 2^17, Null Field Output @18bits

/* 
 * 椭球拟合校准参数 (2026-05-07, 基于 mag_raw_data3.txt 1198 样本)
 * 校准公式: mag_cal[i] = (mag_raw[i] - hard_iron[i]) / soft_iron_scale[i]
 * 校准后各轴统一，模值均值 = 0.9972, std = 0.0404
 */
#define MAG_HARD_IRON_X   1361.1f
#define MAG_HARD_IRON_Y   -940.1f
#define MAG_HARD_IRON_Z   -1008.5f
#define MAG_SCALE_X       6735.0f
#define MAG_SCALE_Y       6956.0f
#define MAG_SCALE_Z       7020.5f

/* 基础 SPI 读写 */
void Mag_WriteReg(uint8_t reg, uint8_t value);
void Mag_ReadReg(uint8_t reg, uint8_t *rx_data, uint8_t len);

/* 磁力计设备功能 */
uint8_t Mag_ReadWhoAmI(void);
void    Mag_SoftReset(void);
void    Mag_Init(void);

void    Mag_WaitForMeasurement(void);//这两个是单次测量使用的函数
uint8_t Mag_IsMeasurementDone(void);

void    Mag_ReadData(int32_t *mx, int32_t *my, int32_t *mz);
void    Mag_ApplyCalibration(float *mx, float *my, float *mz);//校准
void    Mag_ReadCalibratedData(float *mx, float *my, float *mz);//读取原始数据并校准(封装)

#endif /* MMC5983_H */
