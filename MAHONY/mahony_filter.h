#ifndef MAHONY_FILTER_H
#define MAHONY_FILTER_H
//ax+13, ay+56, az-184, gx+19, gy+14, gz+1  零偏值
#include "icm42688.h"
#include "MMC5983.h"
#include "math.h"
#include <stdint.h>

typedef struct {
    float w, x, y, z;
} Quaternion;

typedef struct {
    float x, y, z;
} Vector3f;

void IMU_Init(void);
void Print_initialvalue(void);
/************** 四元数基本运算***********/
Quaternion quat_mul(Quaternion q1, Quaternion q2);// 四元数乘法
Quaternion quat_conj(Quaternion q);// 四元数共轭
float quat_norm(Quaternion q);// 求四元数模长
Quaternion quat_normalize(Quaternion q);// 归一化
Vector3f quat_rotate(Quaternion q, Vector3f v);// 用四元数旋转一个三维向量 (纯四元数方式)
Quaternion quaternion_derivative(Quaternion q, Vector3f omega);// --- 四元数微分方程 ---
/************** 四元数基本运算***********/

Quaternion quaternion_rk4(Quaternion q, Vector3f omega, float dt);// --- 四阶龙格-库塔 (RK4) ---

void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw);//四元数转欧拉角 (ENU 顺序: 偏航-俯仰-滚转)

void Mahony_Filter(Quaternion q, Vector3f Gyro, Vector3f Accel, Vector3f Mag, float dt, 
                   float Kp, float Ki, Vector3f *e_int, Vector3f *omega_corrected);//mahony互补滤波
void Get_Angle(float *roll, float *pitch, float *yaw);//弧度制转角度制即原弧度*(180/pi) 此处函数看实际情况取的60

#endif /* MAHONY_FILTER_H */
