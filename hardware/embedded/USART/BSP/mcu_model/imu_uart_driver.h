#ifndef IMU_UART_DRIVER_H
#define IMU_UART_DRIVER_H

#include <stdint.h>
#include "delay.h"
#include "usart.h"

/* 配置项 / Config */
#ifndef IMU_UART_RX_BUF_SIZE
#define IMU_UART_RX_BUF_SIZE 256  /* 环形接收缓冲区大小 / RX ring buffer size */
#endif

#define FRAME_HEAD1 0x7E
#define FRAME_HEAD2 0x23

/* 功能码 / Function codes */
#define IMU_FUNC_VERSION        0x01
#define IMU_FUNC_RAW_ACCEL      0x04
#define IMU_FUNC_RAW_GYRO       0x0A
#define IMU_FUNC_RAW_MAG        0x10
#define IMU_FUNC_QUAT           0x16
#define IMU_FUNC_EULER          0x26
#define IMU_FUNC_BARO           0x32
#define IMU_FUNC_CALIB_IMU      0x70
#define IMU_FUNC_CALIB_MAG      0x71
#define IMU_FUNC_CALIB_BARO     0x72
#define IMU_FUNC_CALIB_TEMP     0x73
#define IMU_FUNC_REQUEST_DATA   0x80
#define IMU_FUNC_RETURN_STATE   0x81
#define IMU_FUNC_RESET_FLASH    0xA0

/* IMU 实例枚举 / IMU Instance Enumeration */
typedef enum {
    IMU_INSTANCE_1 = 0,  /* 使用 UART2 (PA2/PA3) */
    IMU_INSTANCE_2 = 1,  /* 使用 UART3 (PB10/PB11) */
    IMU_INSTANCE_MAX
} ImuInstance;

/* 结构体：一次性获取所有传感器数据 / Struct: get all sensor data at once */
typedef struct {
    float accel[3];
    float gyro[3];
    float mag[3];
    float quat[4];
    float euler[3];
    float baro[4];
    char  version[8];
} imu_measurement_t;

/* IMU 缓存数据结构 */
#define IMU_CACHE_SIZE 20
typedef struct {
    float pitch;
    float roll;
    float yaw;
} IMU_CACHE_DATA;

/* IMU 硬件配置结构 / IMU Hardware Configuration Structure */
typedef struct {
    USART_TypeDef* usart;           /* USART 外设指针 */
    GPIO_TypeDef*  tx_gpio_port;    /* TX 引脚 GPIO 端口 */
    uint16_t       tx_gpio_pin;     /* TX 引脚编号 */
    GPIO_TypeDef*  rx_gpio_port;    /* RX 引脚 GPIO 端口 */
    uint16_t       rx_gpio_pin;     /* RX 引脚编号 */
    uint32_t       rcc_apb2_gpio;   /* GPIO 时钟 (RCC_APB2Periph_...) */
    uint32_t       rcc_apb1_usart;  /* USART 时钟 (RCC_APB1Periph_...) */
    IRQn_Type      irqn;             /* 中断向量号 */
    uint8_t        irq_preemption_priority; /* 中断抢占优先级 */
    uint8_t        irq_sub_priority;        /* 中断子优先级 */
} ImuHwConfig;

/* IMU 句柄结构 / IMU Handle Structure (封装所有实例相关状态) */
typedef struct {
    /* 硬件配置 */
    const ImuHwConfig* hw_config;
    
    /* 环形缓冲 / RX ring buffer */
    volatile uint8_t  rx_buffer[IMU_UART_RX_BUF_SIZE];
    volatile uint16_t rx_write_index;
    volatile uint16_t rx_read_index;
    
    /* 解析状态机 / Parse state machine */
    uint8_t  rx_state;
    uint8_t  frame_length;
    uint8_t  frame_function;
    uint8_t  frame_buffer[64];
    uint16_t frame_index;
    
    /* 内部缓存数据 / Internal cached state */
    volatile float ax, ay, az;
    volatile float gx, gy, gz;
    volatile float mx, my, mz;
    volatile float roll, pitch, yaw;
    volatile float q0, q1, q2, q3;
    volatile float height, temperature, pressure, pressure_contrast;
    volatile int   version_high, version_mid, version_low;
    volatile uint8_t last_rx_function;
    volatile int16_t last_rx_state;
    
    /* IMU 缓存 / IMU Cache */
    IMU_CACHE_DATA imu_cache[IMU_CACHE_SIZE];
    volatile uint8_t imu_cache_index;
    volatile uint8_t imu_cache_full;
} ImuHandle;

/* 外部声明 IMU 句柄 / External IMU Handles */
extern ImuHandle imu_handles[IMU_INSTANCE_MAX];

/* ---------------- 函数接口 (所有接口增加 ImuInstance 参数) ---------------- */

/**
 * @brief 初始化指定 IMU 实例
 * @param id IMU 实例编号 (IMU_INSTANCE_1 或 IMU_INSTANCE_2)
 */
void IMU_UART_Init(ImuInstance id);

/**
 * @brief 中断接收入口，将新数据写入环形缓冲
 * @param id IMU 实例编号
 * @param data 接收数据指针
 * @param len 数据长度
 */
void IMU_UART_RxBytes(ImuInstance id, volatile uint8_t *data, uint16_t len);

/**
 * @brief 解析环形缓冲中的数据，提取完整帧并更新缓存
 * @param id IMU 实例编号
 */
void IMU_UART_Process(ImuInstance id);

/**
 * @brief 发送命令到指定 IMU
 * @param id IMU 实例编号
 * @param function 功能码
 * @param params 参数指针
 * @param param_len 参数长度
 * @return 0 成功, -1 失败
 */
int IMU_UART_SendCommand(ImuInstance id, uint8_t function, const uint8_t *params, uint8_t param_len);

/**
 * @brief 清理缓存数据
 * @param id IMU 实例编号
 */
void IMU_UART_ClearAutoReportData(ImuInstance id);

/* 数据读取接口 (均增加 ImuInstance 参数) */
int IMU_UART_GetAccelerometer(ImuInstance id, float out[3]);
int IMU_UART_GetGyroscope(ImuInstance id, float out[3]);
int IMU_UART_GetMagnetometer(ImuInstance id, float out[3]);
int IMU_UART_GetQuaternion(ImuInstance id, float out[4]);
int IMU_UART_GetEuler(ImuInstance id, float out[3]);
int IMU_UART_GetBarometer(ImuInstance id, float out[4]);
void IMU_UART_GetVersion(ImuInstance id);
int IMU_UART_GetAll(ImuInstance id, imu_measurement_t *out);

/* 发送函数 (增加 ImuInstance 参数) */
void Send_IMU_Data(ImuInstance id, uint8_t Data);
void Send_IMU_Array(ImuInstance id, uint8_t *pData, uint8_t Length);

/* 校准 API (增加 ImuInstance 参数) */
int IMU_UART_CalibrationImu(ImuInstance id);
int IMU_UART_CalibrationMag(ImuInstance id);
int IMU_UART_CalibrationTemp(ImuInstance id, float now_temperature);
int IMU_UART_ResetUserData(ImuInstance id);
int IMU_UART_WaitCalibration(ImuInstance id, uint8_t function, uint32_t timeout_ms);

/* IMU 缓存操作 (增加 ImuInstance 参数) */
void IMU_Buffer_Push(ImuInstance id);

/* 中断处理函数声明 */
void USART2_IRQHandler(void); /* IMU_INSTANCE_1 中断 */
void USART3_IRQHandler(void); /* IMU_INSTANCE_2 中断 */

#endif /* IMU_UART_DRIVER_H */
