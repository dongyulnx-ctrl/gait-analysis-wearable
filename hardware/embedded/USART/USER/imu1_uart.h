#ifndef __IMU1_UART_H
#define __IMU1_UART_H

#include "stm32f10x.h"
#include "imu_uart_driver.h" // 为了使用 imu_measurement_t 结构体

// 将函数名加前缀 IMU1_ 以区分
void IMU1_UART_Init(void);
void IMU1_UART_Process(void);
void IMU1_UART_GetAll(imu_measurement_t *out);

#endif
