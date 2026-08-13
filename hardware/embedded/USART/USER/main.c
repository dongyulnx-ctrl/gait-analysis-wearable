#include "AllHeader.h"
#include "imu_uart_driver.h"
#include "imu_feature.h"
#include "emg_feature.h"
#include "fusion_send.h"
#include "adc.h"  /* 添加：包含ADC头文件 */
#include <stdio.h>

/* 定义两个 IMU 数据结构体 */
imu_measurement_t imu1_data;
imu_measurement_t imu2_data;


/* 定义两个 ADC 原始值变量 */
uint16_t adc1_value; /* PA4 */
uint16_t adc2_value; /* PA5 */

/* 100ms 发送标志 */
extern volatile uint8_t send_flag;

int main(void)
{
    bsp_init();
    
    /* 初始化定时器 */
    TIM3_Init(); 


    /* 初始化两个 IMU */
    printf("Initializing IMUs...\r\n");
    IMU_UART_Init(IMU_INSTANCE_1);
    IMU_UART_Init(IMU_INSTANCE_2);
    
    /* 初始化 EMG */
    printf("Initializing EMG...\r\n");
    EMG_UART_Init();
	
	   printf("Initializing ADC...\r\n");
    Adc_Config(); 
    
    /* 初始化 Raw 数据模块 */
    IMU_Raw_Init();
    EMG_Raw_Init();

    /* 获取 IMU 版本号 */
    printf("Reading IMU Versions...\r\n");
    IMU_UART_GetVersion(IMU_INSTANCE_1);
    IMU_UART_GetVersion(IMU_INSTANCE_2);

    printf("System Ready! Starting Data Fusion...\r\n");

    while(1)
    {
        /* 1. 解析 IMU 和 EMG */
        IMU_UART_Process(IMU_INSTANCE_1);
        IMU_UART_Process(IMU_INSTANCE_2);
        EMG_UART_Process();

        /* 2. 获取 IMU 数据 */
        IMU_UART_GetAll(IMU_INSTANCE_1, &imu1_data);
        IMU_UART_GetAll(IMU_INSTANCE_2, &imu2_data);

				Get_Two_ADC_Values(&adc1_value, &adc2_value);

        /* 4. 更新 Raw 数据 */
        IMU_Raw_Update(&imu1_data, &imu2_data);

        /* 5. 100ms 发送 */
        if(send_flag)
        {
            Fusion_Send_Update(adc1_value, adc2_value); /* 修改：传入ADC值 */
        }
    }
}