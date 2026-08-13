#include "fusion_send.h"
#include "imu_feature.h"
#include "emg_feature.h"
#include "bsp_timer.h"
#include "stm32f10x.h"
#include "stm32f10x_usart.h" // 【关键修复】明确包含 USART 头文件
#include <string.h>

/* ---------------- 外部变量声明 ---------------- */
extern volatile uint32_t system_time_ms;
extern volatile uint8_t send_flag;

/* ---------------- 内部宏定义 ---------------- */
#define PACK_SIZE 10          // 打包次数：10次

/* ---------------- 单次采集数据结构体 ---------------- */
typedef struct {
    uint32_t timestamp;       // 时间戳
    int16_t imu1_ax, imu1_ay, imu1_az;
    int16_t imu1_gx, imu1_gy, imu1_gz;
    int16_t imu1_roll, imu1_pitch, imu1_yaw;
    int16_t imu2_ax, imu2_ay, imu2_az;
    int16_t imu2_gx, imu2_gy, imu2_gz;
    int16_t imu2_roll, imu2_pitch, imu2_yaw;
    uint16_t emg1_raw, emg2_raw;
    uint16_t adc1, adc2;
} SensorData;

/* ---------------- 内部静态变量 ---------------- */
static SensorData data_buffer[PACK_SIZE];  // 数据缓存数组
static uint8_t pack_count = 0;              // 打包计数器

/* ---------------- 内部辅助函数：串口1发送单个字节 ---------------- */
static void USART1_SendByte(uint8_t byte)
{
    // 【关键修复】直接使用标准库的 USART1，无需宏定义
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    USART_SendData(USART1, byte);
}

/* ---------------- 内部辅助函数：串口1发送字符串 ---------------- */
static void USART1_SendString(const char *str)
{
    while (*str != '\0') {
        USART1_SendByte((uint8_t)*str);
        str++;
    }
}

/* ---------------- 内部辅助函数：串口1发送数字（整数转字符串） ---------------- */
static void USART1_SendInt(int32_t num)
{
    char buf[16];
    uint8_t i = 0;
    uint8_t is_negative = 0;

    // 处理负数
    if (num < 0) {
        is_negative = 1;
        num = -num;
    }

    // 特殊情况：0
    if (num == 0) {
        USART1_SendByte('0');
        return;
    }

    // 数字转字符串（逆序）
    while (num > 0) {
        buf[i++] = (char)('0' + (num % 10));
        num /= 10;
    }

    // 发送负号
    if (is_negative) {
        USART1_SendByte('-');
    }

    // 逆序发送数字
    while (i > 0) {
        USART1_SendByte(buf[--i]);
    }
}

/* ---------------- 内部辅助函数：发送单组传感器数据 ---------------- */
static void Send_Single_Data(const SensorData *data)
{
    // 发送时间戳
    USART1_SendInt((int32_t)data->timestamp);
    USART1_SendByte(' ');

    // 发送IMU1数据
    USART1_SendInt(data->imu1_ax); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_ay); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_az); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_gx); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_gy); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_gz); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_roll); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_pitch); USART1_SendByte(' ');
    USART1_SendInt(data->imu1_yaw); USART1_SendByte(' ');

    // 发送IMU2数据
    USART1_SendInt(data->imu2_ax); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_ay); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_az); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_gx); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_gy); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_gz); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_roll); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_pitch); USART1_SendByte(' ');
    USART1_SendInt(data->imu2_yaw); USART1_SendByte(' ');

    // 发送EMG数据
    USART1_SendInt((int32_t)data->emg1_raw); USART1_SendByte(' ');
    USART1_SendInt((int32_t)data->emg2_raw); USART1_SendByte(' ');

    // 发送ADC数据
    USART1_SendInt((int32_t)data->adc1); USART1_SendByte(' ');
    USART1_SendInt((int32_t)data->adc2);
}

/* ---------------- 核心修改：10次打包发送函数 ---------------- */
static void Fusion_Send_PackedData(void)
{
    __disable_irq(); // 关中断，防止发送过程被打断

    // 1. 发送帧头
    USART1_SendString("AA ");

    // 2. 循环发送10组数据，组间用空格分隔
    for (uint8_t i = 0; i < PACK_SIZE; i++) {
        Send_Single_Data(&data_buffer[i]);
        if (i < PACK_SIZE - 1) {
            USART1_SendByte(' '); // 组间分隔
        }
    }

    // 3. 发送帧尾
    USART1_SendString("\r\n");

    __enable_irq(); // 开中断
}

/* ---------------- 主函数：100ms触发，存数据，存满10次再发 ---------------- */
void Fusion_Send_Update(uint16_t adc1, uint16_t adc2)
{
    if (send_flag == 0)
        return;

    send_flag = 0;

    /* 1. 获取双IMU原始数据 */
    IMU_RAW_DATA *imu_raw = IMU_Raw_Get();

    /* 2. 获取双EMG原始值 */
    uint16_t emg1_raw, emg2_raw;
    EMG_Raw_Get(&emg1_raw, &emg2_raw);

    /* 3. 将当前数据存入缓存数组（保留原有放大逻辑） */
    SensorData *current_data = &data_buffer[pack_count];
    
    current_data->timestamp = system_time_ms;

    // IMU1 数据放大
    current_data->imu1_ax = (int16_t)(imu_raw->imu1_accel[0] * 1000);
    current_data->imu1_ay = (int16_t)(imu_raw->imu1_accel[1] * 1000);
    current_data->imu1_az = (int16_t)(imu_raw->imu1_accel[2] * 1000);
    current_data->imu1_gx = (int16_t)(imu_raw->imu1_gyro[0] * 1000);
    current_data->imu1_gy = (int16_t)(imu_raw->imu1_gyro[1] * 1000);
    current_data->imu1_gz = (int16_t)(imu_raw->imu1_gyro[2] * 1000);
    current_data->imu1_roll = (int16_t)(imu_raw->imu1_euler[0] * 100);
    current_data->imu1_pitch = (int16_t)(imu_raw->imu1_euler[1] * 100);
    current_data->imu1_yaw = (int16_t)(imu_raw->imu1_euler[2] * 100);

    // IMU2 数据放大
    current_data->imu2_ax = (int16_t)(imu_raw->imu2_accel[0] * 1000);
    current_data->imu2_ay = (int16_t)(imu_raw->imu2_accel[1] * 1000);
    current_data->imu2_az = (int16_t)(imu_raw->imu2_accel[2] * 1000);
    current_data->imu2_gx = (int16_t)(imu_raw->imu2_gyro[0] * 1000);
    current_data->imu2_gy = (int16_t)(imu_raw->imu2_gyro[1] * 1000);
    current_data->imu2_gz = (int16_t)(imu_raw->imu2_gyro[2] * 1000);
    current_data->imu2_roll = (int16_t)(imu_raw->imu2_euler[0] * 100);
    current_data->imu2_pitch = (int16_t)(imu_raw->imu2_euler[1] * 100);
    current_data->imu2_yaw = (int16_t)(imu_raw->imu2_euler[2] * 100);

    // EMG和ADC数据
    current_data->emg1_raw = emg1_raw;
    current_data->emg2_raw = emg2_raw;
    current_data->adc1 = adc1;
    current_data->adc2 = adc2;

    /* 4. 计数器加1 */
    pack_count++;

    /* 5. 存满10次：打包发送，清空计数器 */
    if (pack_count >= PACK_SIZE) {
        Fusion_Send_PackedData();
        pack_count = 0;
    }
}
