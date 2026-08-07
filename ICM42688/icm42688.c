#include "icm42688.h"

// 写单个寄存器
void Gyro_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg & 0x7F;       // 最高位清0表示写
    tx_buf[1] = value;

    GYRO_CS1_LOW();
    HAL_SPI_Transmit(&hspi3, tx_buf, 2, 10);
    GYRO_CS1_HIGH();
}

// 读单个寄存器(或连续读多个)
// ICM42688 要求整个读操作期间时钟完全连续，因此必须用一次 TransmitReceive
void Gyro_ReadReg(uint8_t reg, uint8_t *rx_data, uint8_t len)
{
    uint8_t tx_buf[7];   // 最大 len 为 6，加 1 字节命令
    uint8_t rx_buf[7];

    tx_buf[0] = reg | 0x80;           // 最高位置1表示读
    for (uint8_t i = 1; i <= len; i++) {
        tx_buf[i] = 0x00;             // dummy 字节，用于产生读取时钟
    }

    GYRO_CS1_LOW();
    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, len + 1, 10);
    GYRO_CS1_HIGH();

    // rx_buf[0] 是命令字节期间收到的无效数据，真正的数据从 rx_buf[1] 开始
    for (uint8_t i = 0; i < len; i++) {
        rx_data[i] = rx_buf[i + 1];
    }
}

uint8_t Icm_ReadWhoAmI(void)
{
    uint8_t whoami = 0;
    Gyro_ReadReg(0x75, &whoami, 1);   // Bank 0, 地址0x75
    return whoami;
}

// Bank 选择寄存器在 0x76，且在所有 Bank 里都能访问
void Icm_SwitchBank(uint8_t bank)
{
    Gyro_WriteReg(0x76, bank & 0x07);
}

void Icm_Init(void)
{
    //--- 1. 软复位 ---

    Gyro_WriteReg(0x11, 0x01);                // DEVICE_CONFIG: soft reset
    HAL_Delay(2);                             // 手册要求等1ms以上
    Icm_SwitchBank(0);

    //--- 2. 配置传感器参数 (必须在传感器关闭状态下配置!) ---
    // 加速度计：±16g 量程, 1kHz 输出数据速率 (ODR)
    // ACCEL_CONFIG0 (0x50)：ACCEL_FS_SEL = 000, ACCEL_ODR = 0110
    Gyro_WriteReg(0x50, 0x06);
    // 陀螺仪：±2000dps 量程, 1kHz 输出数据速率 (ODR)
    // GYRO_CONFIG0 (0x4F)：GYRO_FS_SEL = 000, GYRO_ODR = 0110
    Gyro_WriteReg(0x4F, 0x06);

    // INT_CONFIG1 (0x64): INT_ASYNC_RESET = 0 (手册强制要求)
    Gyro_WriteReg(0x64, 0x00);

    //--- 3. 开启传感器 (最后一步) ---
    // PWR_MGMT0 (0x4E): ACCEL_MODE=11, GYRO_MODE=11 (Low Noise)
    Gyro_WriteReg(0x4E, 0x0F);

    // 等待传感器起振 (手册要求 gyro 至少 30ms, accel 至少 10ms)
    HAL_Delay(50);
}

void Icm_ReadAccel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6];
    Gyro_ReadReg(0x1F, buf, 6);  // 从 ACCEL_DATA_X1 开始连续读6字节

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
}

void Icm_ReadGyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6];
    Gyro_ReadReg(0x25, buf, 6);  // 从 GYRO_DATA_X1 开始连续读6字节

    *gx = (int16_t)((buf[0] << 8) | buf[1]);
    *gy = (int16_t)((buf[2] << 8) | buf[3]);
    *gz = (int16_t)((buf[4] << 8) | buf[5]);
}