#include "imu_feature.h"

/* 静态变量：存储双IMU原始数据副本 */
static IMU_RAW_DATA raw_data;

void IMU_Raw_Init(void)
{
    /* 清空所有数据 */
    for(int i = 0; i < 3; i++)
    {
        raw_data.imu1_accel[i] = 0.0f;
        raw_data.imu1_gyro[i] = 0.0f;
        raw_data.imu1_euler[i] = 0.0f;
        
        raw_data.imu2_accel[i] = 0.0f;
        raw_data.imu2_gyro[i] = 0.0f;
        raw_data.imu2_euler[i] = 0.0f;
    }
}

void IMU_Raw_Update(imu_measurement_t *imu1, imu_measurement_t *imu2)
{
    /* 检查指针是否为空 */
    if(!imu1 || !imu2) return;

    /* 复制 IMU1 数据 */
    raw_data.imu1_accel[0] = imu1->accel[0];
    raw_data.imu1_accel[1] = imu1->accel[1];
    raw_data.imu1_accel[2] = imu1->accel[2];
    
    raw_data.imu1_gyro[0] = imu1->gyro[0];
    raw_data.imu1_gyro[1] = imu1->gyro[1];
    raw_data.imu1_gyro[2] = imu1->gyro[2];
    
    raw_data.imu1_euler[0] = imu1->euler[0];
    raw_data.imu1_euler[1] = imu1->euler[1];
    raw_data.imu1_euler[2] = imu1->euler[2];

    /* 复制 IMU2 数据 */
    raw_data.imu2_accel[0] = imu2->accel[0];
    raw_data.imu2_accel[1] = imu2->accel[1];
    raw_data.imu2_accel[2] = imu2->accel[2];
    
    raw_data.imu2_gyro[0] = imu2->gyro[0];
    raw_data.imu2_gyro[1] = imu2->gyro[1];
    raw_data.imu2_gyro[2] = imu2->gyro[2];
    
    raw_data.imu2_euler[0] = imu2->euler[0];
    raw_data.imu2_euler[1] = imu2->euler[1];
    raw_data.imu2_euler[2] = imu2->euler[2];
}

IMU_RAW_DATA* IMU_Raw_Get(void)
{
    return &raw_data;
}