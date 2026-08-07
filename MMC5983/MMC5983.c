#include "MMC5983.h"

// 写单个寄存器
void Mag_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg & 0x7F;       // 最高位清0表示写
    tx_buf[1] = value;

    MAG_CS2_LOW();
    HAL_SPI_Transmit(&hspi3, tx_buf, 2, 10);
    MAG_CS2_HIGH();
}

// 读单个寄存器(或连续读多个)
void Mag_ReadReg(uint8_t reg, uint8_t *rx_data, uint8_t len)
{
    uint8_t tx_buf[8];   // 最大 len 为 7，加 1 字节命令
    uint8_t rx_buf[8];

    tx_buf[0] = reg | 0x80;           // 最高位置1表示读
    for (uint8_t i = 1; i <= len; i++) {
        tx_buf[i] = 0x00;             // dummy 字节，用于产生读取时钟
    }

    MAG_CS2_LOW();
    HAL_SPI_TransmitReceive(&hspi3, tx_buf, rx_buf, len + 1, 10);
    MAG_CS2_HIGH();

    // rx_buf[0] 是命令字节期间收到的无效数据，真正的数据从 rx_buf[1] 开始
    for (uint8_t i = 0; i < len; i++) {
        rx_data[i] = rx_buf[i + 1];
    }
}

uint8_t Mag_ReadWhoAmI(void)
{
    uint8_t whoami = 0;
    Mag_ReadReg(MMC5983_WHOAMI, &whoami, 1);
    return whoami;
}

void Mag_SoftReset(void)
{
    // 数据手册 Page 15: CTRL1 (0x0A) bit7 = SW_RST, 写 1 触发软复位
    Mag_WriteReg(MMC5983_CTRL1, MMC5983_CTRL1_SW_RST);
    HAL_Delay(15);  // 等待 OTP 重新加载完成 (power-on time 10ms)
}

// 检查测量是否完成 (STATUS 寄存器的 bit 0: Meas_M_Done)
uint8_t Mag_IsMeasurementDone(void)
{
    uint8_t status;
    Mag_ReadReg(MMC5983_STATUS, &status, 1);
    return (status & MMC5983_STATUS_MEAS_M_DONE) != 0;
}

// 等待测量完成 (超时保护)
void Mag_WaitForMeasurement(void)//非连续模式下读值需要才使用的函数
{
    uint32_t timeout = 10000;
    while (!Mag_IsMeasurementDone() && timeout > 0) {
        timeout--;
    }
}

void Mag_Init(void)
{
    // 1. 软复位，等待 OTP 加载
    Mag_SoftReset();

    // 2. CTRL1 (0x0A): BW[1:0]=11 (800Hz 带宽, 0.5ms 测量时间)
    //    不禁止任何轴 (X/Y/Z-inhibit=0), SW_RST=0
    //    数据手册 Page 15: BW=11 → 800Hz, 配合 CM_Freq=111 可达 1000Hz ODR
    Mag_WriteReg(MMC5983_CTRL1, 0x03);

    // 3. 手动执行一次 SET/RESET 消磁, 必须在单次模式下分步完成
    //    步骤3a: 触发 SET
    //    CTRL0 bit3=Set=0x08 (自清除, 脉冲约 500ns)
    Mag_WriteReg(MMC5983_CTRL0, 0x08);
    HAL_Delay(1);

    //    步骤3b: 触发 RESET
    //    CTRL0 bit4=Reset=0x10
    //    写 0 再写 0x10 产生上升沿以触发
    Mag_WriteReg(MMC5983_CTRL0, 0x00);
    HAL_Delay(1);
    Mag_WriteReg(MMC5983_CTRL0, 0x10);
    HAL_Delay(1);

    // 4. 关闭 TM_M, 开启 Auto_SR (bit5=0x20)
    //    此后每次测量由硬件自动执行 SET/RESET
    Mag_WriteReg(MMC5983_CTRL0, 0x20);

    // 5. CTRL2 (0x0B) 配置 1kHz 连续模式:
    //    bit7=1      En_prd_set  使能周期性 SET
    //    bit[6:4]=001 Prd_set     每 25 次测量执行一次 SET (兼顾速度与精度)
    //    bit3=1      Cmm_en      使能连续测量模式
    //    bit[2:0]=111 CM_Freq     1000Hz (配合 BW=11)
    //    → 0b10011111 = 0x9F
    Mag_WriteReg(MMC5983_CTRL2, 0x9F);
}

void Mag_ReadData(int32_t *mx, int32_t *my, int32_t *mz)
{
    uint8_t buf[7];
    uint32_t raw_x18, raw_y18, raw_z18;

    // 从 XOUT0 开始连续读 7 字节
    Mag_ReadReg(MMC5983_XOUT0, buf, 7);

    // 拼接 18 位原始值 (无符号)
    // 数据手册 Page 13-14:
    //   XOUT0 = X[17:10] (高字节), XOUT1 = X[9:2] (中字节)
    //   XYZOUT2 bit[7:6] = X[1:0]
    raw_x18  = ((uint32_t)buf[0] << 10);           // X[17:10] → bit[17:10]
    raw_x18 |= ((uint32_t)buf[1] << 2);             // X[9:2]   → bit[9:2]
    raw_x18 |= ((uint32_t)(buf[6] >> 6) & 0x03);   // X[1:0]   → bit[1:0]

    //   YOUT0 = Y[17:10], YOUT1 = Y[9:2]
    //   XYZOUT2 bit[5:4] = Y[1:0]
    raw_y18  = ((uint32_t)buf[2] << 10);
    raw_y18 |= ((uint32_t)buf[3] << 2);
    raw_y18 |= ((uint32_t)(buf[6] >> 4) & 0x03);

    //   ZOUT0 = Z[17:10], ZOUT1 = Z[9:2]
    //   XYZOUT2 bit[3:2] = Z[1:0]
    raw_z18  = ((uint32_t)buf[4] << 10);
    raw_z18 |= ((uint32_t)buf[5] << 2);
    raw_z18 |= ((uint32_t)(buf[6] >> 2) & 0x03);

    // 18 位无符号 → 有符号转换 (Null Field Output = 131072 = 2^17)
    *mx = (int32_t)(raw_x18 - MMC5983_18BIT_ZERO);
    *my = (int32_t)(raw_y18 - MMC5983_18BIT_ZERO);
    *mz = -(int32_t)(raw_z18 - MMC5983_18BIT_ZERO);//取反将磁力计和陀螺仪轴正反向对齐
}

/**
 * @brief  对磁力计原始有符号值进行硬铁/软铁校准
 * @param  mx: 输入原始值(float可接受int32_t隐式转换), 输出校准后值
 * @param  my: 同上
 * @param  mz: 同上
 * @note   校准公式: cal = (raw - hard_iron) / scale
 *         校准后各轴磁场模值均值为 ±1(相对单位)
 */
void Mag_ApplyCalibration(float *mx, float *my, float *mz)
{
    *mx = ((*mx) - MAG_HARD_IRON_X) / MAG_SCALE_X;
    *my = ((*my) - MAG_HARD_IRON_Y) / MAG_SCALE_Y;
    *mz = ((*mz) - MAG_HARD_IRON_Z) / MAG_SCALE_Z;
}

/**
 * @brief  读取磁力计原始数据并直接校准，一步封装
 * @param  mx: 输出校准后的X轴值 (float)
 * @param  my: 输出校准后的Y轴值 (float)
 * @param  mz: 输出校准后的Z轴值 (float)
 */
void Mag_ReadCalibratedData(float *mx, float *my, float *mz)
{
    int32_t raw_x, raw_y, raw_z;
    Mag_ReadData(&raw_x, &raw_y, &raw_z);
    *mx = (float)raw_x;
    *my = (float)raw_y;
    *mz = (float)raw_z;
    Mag_ApplyCalibration(mx, my, mz);
}
