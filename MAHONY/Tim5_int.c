#include "mahony_filter.h"
#include <math.h>
#include <stdint.h>

#define GYRO_SENSITIVITY (2000.0f / 32768.0f)  // dps per LSB
#define DPS_TO_RADS (M_PI / 180.0f)            // dps → rad/s


int tim=0;//定时中断测试变量
Quaternion q = {1.0f, 0.0f, 0.0f, 0.0f};  // 初始化为单位四元数
Vector3f e_int = {0.0f, 0.0f, 0.0f};  // 误差积分项
Vector3f gyro, acc, mag;               // 传感器数据
Vector3f omega_corrected;              // 纠正后的角速度

float Kp = 5.0f;   // Mahony 比例系数 5.0
float Ki = 0.3f; // Mahony 积分系数   0.3
float dt = 0.001f; // 采样时间 (s)

int16_t ax, ay, az, gx, gy, gz;
float mx, my, mz;
extern float pitch, roll, yaw;
float isr_time_us;       // ISR 耗时 (微秒)，供 main.c 打印
uint32_t isr_max_cycles; // ISR 最大周期数

float yaw_offset = 0.0f;              // 上电初始偏置
int yaw_offset_locked = 0;            // 偏置锁定标志
const int yaw_offset_samples = 5000;  // 锁定前采样数




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        tim++;//以中断频率++

        // uint32_t t_start = DWT->CYCCNT;  // 记录起始时刻  /*四元数mahony算法运行时间测试语句*/


        // 1. 读取传感器原始值
        Icm_ReadAccel(&ax, &ay, &az);
        Icm_ReadGyro(&gx, &gy, &gz);
        Mag_ReadCalibratedData(&mx, &my, &mz);

        // 2. 单位转换（LSB → rad/s）+ 零偏消除（必须在滤波之前）
        gyro.x = (gx + 17) * GYRO_SENSITIVITY * DPS_TO_RADS;
        gyro.y = (gy + 15) * GYRO_SENSITIVITY * DPS_TO_RADS;
       
        // if (gz == 255) {//消掉跳变
        // gyro.z = 0;
        // }
        // else {
        gyro.z = (gz -1) * GYRO_SENSITIVITY * DPS_TO_RADS;/* 第er个陀螺仪角速度漂移值   gyro.x = (gx + 17) * GYRO_SENSITIVITY * DPS_TO_RADS;
                                                                                        gyro.y = (gy + 15) * GYRO_SENSITIVITY * DPS_TO_RADS;
                                                                                        gyro.z = (gz + 12) * GYRO_SENSITIVITY * DPS_TO_RADS;*/
        // }

        acc.x = (ax + 8);
        acc.y = (ay + 64);
        acc.z = (az - 182);  /* 第二个陀螺仪加速度漂移值 acc.x = (ax - 40);
                                                        acc.y = (ay + 64);
                                                        acc.z = (az - 182);*/

        mag.x = mx;
        mag.y = my;
        mag.z = mz;

        // 3. Mahony 滤波
        Mahony_Filter(q, gyro, acc, mag, dt, Kp, Ki, &e_int, &omega_corrected);

        //4. RK4 积分(函数最后重新自带归一化来消除模长偏移)
        q = quaternion_rk4(q, omega_corrected, dt);
        
        //四元数转欧拉角（输出弧度）
        quaternion_to_euler(q, &roll, &pitch, &yaw);

        // --- yaw 上电归零：前 yaw_offset_samples 次采样锁定偏置 ---
        if (!yaw_offset_locked) {
            static int sample_cnt = 0;
            if (sample_cnt < yaw_offset_samples) {
                sample_cnt++;
                if (sample_cnt == yaw_offset_samples) {
                    yaw_offset = yaw;            // 锁定当前绝对 yaw
                    yaw_offset_locked = 1;
                }
            }
        }

        // 减去偏置，得到相对角度
        yaw -= yaw_offset;

        // 归一化到 [-π, π]
        while (yaw >  M_PI) yaw -= 2.0f * M_PI;
        while (yaw < -M_PI) yaw += 2.0f * M_PI;


        
        // quaternion_to_euler 输出是弧度，转成角度（后给pid用）范围 [-180, 180]
        // 误差计算（角度单位统一）
        // float error = Angle_target - yaw_deg;
        // if (error > 180.0f)  error -= 360.0f;  // 走最短路径
        // if (error < -180.0f) error += 360.0f;
        

        // uint32_t t_end = DWT->CYCCNT;                         /*四元数mahony算法运行时间测试语句*/
        // uint32_t cycles = t_end - t_start;                    /*四元数mahony算法运行时间测试语句*/
        // isr_time_us = cycles / 168.0f;                        /*四元数mahony算法运行时间测试语句*/
        // if (cycles > isr_max_cycles) isr_max_cycles = cycles; /*四元数mahony算法运行时间测试语句*/
    }
}