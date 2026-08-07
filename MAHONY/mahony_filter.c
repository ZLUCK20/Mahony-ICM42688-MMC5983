#include "mahony_filter.h"
#include "usart.h"
#include <math.h>
#include <stdio.h>

#define PERCENT 0  // 磁力计误差参与融合，稳定 yaw
void IMU_Init(void)
{
  static  uint8_t id1=0;
  static  uint8_t id2=0;
  //icm42688上电初始化
  id1 = Icm_ReadWhoAmI();
  printf("Gyro_WHO_AM_I = 0x%02X (expected: 0x47)\r\n", id1);
  Icm_Init();
  //随后MCC5983上电初始化
  id2 = Mag_ReadWhoAmI();
  printf("Mag_WHO_AM_I = 0x%02X (expected: 0x30)\r\n", id2);
  Mag_Init();
}

void Print_initialvalue(void)
{
  int16_t Ax_Init, Ay_Init, Az_Init, Wx_Init, Wy_Init, Wz_Init;
  float mx_Init, my_Init, mz_Init;
  Icm_ReadAccel(&Ax_Init, &Ay_Init, &Az_Init);
  Icm_ReadGyro(&Wx_Init, &Wy_Init, &Wz_Init);
  Mag_ReadCalibratedData(&mx_Init, &my_Init, &mz_Init);
  printf("Accel: %6d %6d %6d | Gyro: %6d %6d %6d\r\n", 
          Ax_Init, Ay_Init, Az_Init, Wx_Init, Wy_Init, Wz_Init);
  printf("Mag: %f %f %f\r\n", mx_Init, my_Init, mz_Init);
}

/************** 四元数基本运算***********/
// 四元数乘法
Quaternion quat_mul(Quaternion q1, Quaternion q2) {                      
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

// 四元数共轭
Quaternion quat_conj(Quaternion q) {
    Quaternion res = {q.w, -q.x, -q.y, -q.z};
    return res;
}

// 求四元数模长
float quat_norm(Quaternion q) {
    return sqrtf(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
}

// 归一化
Quaternion quat_normalize(Quaternion q) {
    float n = quat_norm(q);
    if (n < 1e-6f) return q;
    Quaternion res = {q.w/n, q.x/n, q.y/n, q.z/n};
    return res;
}

// 用四元数旋转一个三维向量 (纯四元数方式)
Vector3f quat_rotate(Quaternion q, Vector3f v) {
    Quaternion p = {0.0f, v.x, v.y, v.z};
    Quaternion q_inv = quat_conj(q);   // 单位四元数的逆 = 共轭
    Quaternion rotated = quat_mul(quat_mul(q, p), q_inv);
    Vector3f res = {rotated.x, rotated.y, rotated.z};
    return res;
}

// ---------- 四元数微分方程 ----------
// 输入: 当前四元数 q，角速度向量 omega (rad/s, 载体坐标系)
// 输出: 四元数导数 dq/dt = (1/2) * q ⊗ (0, omega)
Quaternion quaternion_derivative(Quaternion q, Vector3f omega) {
    Quaternion omega_q = {0.0f, omega.x, omega.y, omega.z};
    Quaternion dq = quat_mul(q, omega_q);
    dq.w *= 0.5f; dq.x *= 0.5f; dq.y *= 0.5f; dq.z *= 0.5f;
    return dq;
}

/**************四阶龙格-库塔 (RK4)***********/
// 输入: 当前四元数 q，角速度向量 omega (修正后的，rad/s)，时间步长 dt
// 输出: 更新后的四元数
Quaternion quaternion_rk4(Quaternion q, Vector3f omega, float dt) {
    Quaternion k1, k2, k3, k4;
    Quaternion q_tmp;
    float half_dt = 0.5f * dt;

    // k1 = f(q, omega)
    k1 = quaternion_derivative(q, omega);

    // k2 = f(q + 0.5*dt*k1, omega)
    q_tmp.w = q.w + half_dt * k1.w;
    q_tmp.x = q.x + half_dt * k1.x;
    q_tmp.y = q.y + half_dt * k1.y;
    q_tmp.z = q.z + half_dt * k1.z;
    k2 = quaternion_derivative(q_tmp, omega);

    // k3 = f(q + 0.5*dt*k2, omega)
    q_tmp.w = q.w + half_dt * k2.w;
    q_tmp.x = q.x + half_dt * k2.x;
    q_tmp.y = q.y + half_dt * k2.y;
    q_tmp.z = q.z + half_dt * k2.z;
    k3 = quaternion_derivative(q_tmp, omega);

    // k4 = f(q + dt*k3, omega)
    q_tmp.w = q.w + dt * k3.w;
    q_tmp.x = q.x + dt * k3.x;
    q_tmp.y = q.y + dt * k3.y;
    q_tmp.z = q.z + dt * k3.z;
    k4 = quaternion_derivative(q_tmp, omega);

    // q_new = q + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
    Quaternion q_new;
    q_new.w = q.w + dt/6.0f * (k1.w + 2.0f*k2.w + 2.0f*k3.w + k4.w);
    q_new.x = q.x + dt/6.0f * (k1.x + 2.0f*k2.x + 2.0f*k3.x + k4.x);
    q_new.y = q.y + dt/6.0f * (k1.y + 2.0f*k2.y + 2.0f*k3.y + k4.y);
    q_new.z = q.z + dt/6.0f * (k1.z + 2.0f*k2.z + 2.0f*k3.z + k4.z);
    q_new = quat_normalize(q_new);
    return q_new;
}

// ---------- (可选) 四元数转欧拉角 (NWU 顺序: 偏航-俯仰-滚转) ----------
// 返回: 欧拉角 (roll, pitch, yaw) 单位: 弧度
void quaternion_to_euler(Quaternion q, float *roll, float *pitch, float *yaw) {
    float yaw_val   = atan2f(2.0f*(q.w*q.z + q.x*q.y), 1.0f - 2.0f*(q.y*q.y + q.z*q.z));
    float pitch_val = 2.0f*(q.w*q.y - q.z*q.x);
    // 钳位 asinf 参数在 [-1, 1] 范围内，防止浮点累积误差导致 NaN
    if (pitch_val >  1.0f) pitch_val =  1.0f;
    if (pitch_val < -1.0f) pitch_val = -1.0f;
    float pitch_rad = asinf(pitch_val);
    float roll_val  = atan2f(2.0f*(q.w*q.x + q.y*q.z), 1.0f - 2.0f*(q.x*q.x + q.y*q.y));
    *yaw   = yaw_val;
    *pitch = pitch_rad;
    *roll  = roll_val;
}


/*mahony_filter函数*/
// 输入/输出:
//   q           : 当前姿态四元数
//   gyro        : 陀螺仪原始角速度 (rad/s, 载体坐标系)
//   acc         : 加速度计测量值 (单位: g, 已归一化)
//   mag         : 磁力计测量值 (单位: 任意，但方向已校准, 载体坐标系)
//   dt          : 采样时间间隔 (s)
//   Kp, Ki      : PI 参数 (比例、积分系数)
//   e_int       : 积分误差项指针 (三维向量, 需持久存储)
// 输出:
//   omega_corrected : 修正后的角速度 (rad/s, 载体坐标系)
void Mahony_Filter(Quaternion q, Vector3f Gyro, Vector3f Accel, Vector3f Mag, float dt, 
                   float Kp, float Ki, Vector3f *e_int, Vector3f *omega_corrected)
{
    //加速度归一化
    float acc_norm = sqrtf(Accel.x * Accel.x + Accel.y * Accel.y + Accel.z * Accel.z);
    if (acc_norm > 1e-6f) {
        Accel.x /= acc_norm; Accel.y /= acc_norm; Accel.z /= acc_norm;
    }
    //磁力计已经归一化了，这里就不做处理了 直接使用传进来的Mag.x  y  z值

    //加速度误差（叉积计算）
    Vector3f world_gravity = {0.0f, 0.0f, 1.0f};
    Vector3f v = quat_rotate(quat_conj(q), world_gravity);  /*将世界系重力抓换到载体系v = q^{-1} * (0,0,1) * q*/
    Vector3f e_acc;
    e_acc.x = Accel.y * v.z - Accel.z * v.y;
    e_acc.y = Accel.z * v.x - Accel.x * v.z;
    e_acc.z = Accel.x * v.y - Accel.y * v.x;
    
    // ---------- 3. 磁力计误差 (NWU: 北=X, 西=Y, 天=Z) ----------
    //载体系磁场转世界系
    Vector3f h = quat_rotate(q, Mag);
    // 水平投影强度 (倾角补偿用，不是归一化)
    float b = sqrtf(h.x*h.x + h.y*h.y);
    // NWU 期望磁场: 水平分量全指北(+X)
    Vector3f ref_mag_world = {b, 0.0f, h.z};
    //期望磁场转回载体系
    Vector3f w = quat_rotate(quat_conj(q), ref_mag_world);
    //叉积求磁力计误差
    Vector3f e_mag;
    e_mag.x = Mag.y * w.z - Mag.z * w.y;
    e_mag.y = Mag.z * w.x - Mag.x * w.z;
    e_mag.z = Mag.x * w.y - Mag.y * w.x;
    //误差总和
    Vector3f e_total;
    e_total.x = e_acc.x + e_mag.x * PERCENT;
    e_total.y = e_acc.y + e_mag.y * PERCENT;
    e_total.z = e_acc.z + e_mag.z * PERCENT;

    // 积分项累加 (用于消除陀螺仪常值零偏)
    e_int->x += e_total.x * dt;
    e_int->y += e_total.y * dt;
    e_int->z += e_total.z * dt;
    // 防止积分饱和 (角速度修正量不应超过 ~MAHONY_INTEGRAL_LIMIT rad/s)
    #define MAHONY_INTEGRAL_LIMIT 10.0f
    if (e_int->x >  MAHONY_INTEGRAL_LIMIT) e_int->x =  MAHONY_INTEGRAL_LIMIT;
    if (e_int->x < -MAHONY_INTEGRAL_LIMIT) e_int->x = -MAHONY_INTEGRAL_LIMIT;
    if (e_int->y >  MAHONY_INTEGRAL_LIMIT) e_int->y =  MAHONY_INTEGRAL_LIMIT;
    if (e_int->y < -MAHONY_INTEGRAL_LIMIT) e_int->y = -MAHONY_INTEGRAL_LIMIT;
    if (e_int->z >  MAHONY_INTEGRAL_LIMIT) e_int->z =  MAHONY_INTEGRAL_LIMIT;
    if (e_int->z < -MAHONY_INTEGRAL_LIMIT) e_int->z = -MAHONY_INTEGRAL_LIMIT;

    omega_corrected->x = Gyro.x + Kp * e_total.x + Ki * e_int->x;
    omega_corrected->y = Gyro.y + Kp * e_total.y + Ki * e_int->y;
    omega_corrected->z = Gyro.z + Kp * e_total.z + Ki * e_int->z;

}

void Get_Angle(float *pitch, float *roll, float *yaw)//弧度制转角度制 = 弧度 * (180/π)
{
    // 使用临时变量避免重复调用时在原值上累积乘法
    float p = *pitch, r = *roll, y = *yaw;
    // 检查 NaN（NaN 在任何比较中都返回 false，包括 NaN==NaN）
    if (!(p == p)) p = 0.0f;
    if (!(r == r)) r = 0.0f;
    if (!(y == y)) y = 0.0f;
    *pitch = p * 57.2958f;
    *yaw   = y * 57.2958f;
    *roll  = r * 57.2958f;
}


