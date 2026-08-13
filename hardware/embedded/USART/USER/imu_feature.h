#ifndef __IMU_FEATURE_H
#define __IMU_FEATURE_H

#include "imu_uart_driver.h"

/* 简化后的结构体：只存双IMU的原始数据副本 */
typedef struct {
    /* IMU1 原始数据 */
    float imu1_accel[3];  /* ax, ay, az (g) */
    float imu1_gyro[3];   /* gx, gy, gz (rad/s) */
    float imu1_euler[3];  /* roll, pitch, yaw (deg) */
    
    /* IMU2 原始数据 */
    float imu2_accel[3];
    float imu2_gyro[3];
    float imu2_euler[3];
} IMU_RAW_DATA;

/* 初始化（清空数据） */
void IMU_Raw_Init(void);

/* 更新双IMU原始数据（传入两个imu_measurement_t指针） */
void IMU_Raw_Update(imu_measurement_t *imu1, imu_measurement_t *imu2);

/* 获取双IMU原始数据指针 */
IMU_RAW_DATA* IMU_Raw_Get(void);

#endif /* __IMU_FEATURE_H */