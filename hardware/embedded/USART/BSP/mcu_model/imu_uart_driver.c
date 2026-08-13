#include "imu_uart_driver.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static const ImuHwConfig hw_configs[IMU_INSTANCE_MAX] = {
    /* IMU_INSTANCE_1: UART2, PA2(TX), PA3(RX) */
    {
        .usart = USART2,
        .tx_gpio_port = GPIOA,
        .tx_gpio_pin = GPIO_Pin_2,
        .rx_gpio_port = GPIOA,
        .rx_gpio_pin = GPIO_Pin_3,
        .rcc_apb2_gpio = RCC_APB2Periph_GPIOA,
        .rcc_apb1_usart = RCC_APB1Periph_USART2,
        .irqn = USART2_IRQn,
        .irq_preemption_priority = 1,  // 不变
        .irq_sub_priority = 0          // 改 0
    },
    /* IMU_INSTANCE_2: UART3, PB10(TX), PB11(RX) */
    {
        .usart = USART3,
        .tx_gpio_port = GPIOB,
        .tx_gpio_pin = GPIO_Pin_10,
        .rx_gpio_port = GPIOB,
        .rx_gpio_pin = GPIO_Pin_11,
        .rcc_apb2_gpio = RCC_APB2Periph_GPIOB,
        .rcc_apb1_usart = RCC_APB1Periph_USART3,
        .irqn = USART3_IRQn,
        .irq_preemption_priority = 2,  // 改 2
        .irq_sub_priority = 0          // 改 0
    }
};

/* ---------------- IMU 句柄实例 ---------------- */
ImuHandle imu_handles[IMU_INSTANCE_MAX] = {0};

/* ---------------- 内部辅助宏/函数 (操作句柄内的环形缓冲) ---------------- */
static inline uint16_t _rxbuf_next(ImuHandle* handle, uint16_t index)
{
    return (uint16_t)((index + 1u) % IMU_UART_RX_BUF_SIZE);
}

static inline int _rxbuf_is_empty(ImuHandle* handle)
{
    return handle->rx_write_index == handle->rx_read_index;
}

static inline void _rxbuf_push(ImuHandle* handle, uint8_t byte_value)
{
    uint16_t next_index = _rxbuf_next(handle, handle->rx_write_index);
    if (next_index == handle->rx_read_index) {
        handle->rx_read_index = _rxbuf_next(handle, handle->rx_read_index);
    }
    handle->rx_buffer[handle->rx_write_index] = byte_value;
    handle->rx_write_index = next_index;
}

static inline int _rxbuf_pop(ImuHandle* handle, uint8_t *out_byte)
{
    if (_rxbuf_is_empty(handle)) {
        return -1;
    }
    *out_byte = handle->rx_buffer[handle->rx_read_index];
    handle->rx_read_index = _rxbuf_next(handle, handle->rx_read_index);
    return 0;
}

/** 将两个字节转换为 int16 */
static int16_t to_int16(const uint8_t *bytes)
{
    return (int16_t)((bytes[1] << 8) + bytes[0]);
}

/** 将四个字节转换为 float */
static float to_float(const uint8_t *bytes)
{
    float v;
    memcpy(&v, bytes, sizeof(float));
    return v;
}

/* ---------------- 解析数据帧 (操作指定句柄) ---------------- */
static void _parse_frame_data(ImuHandle* handle, uint8_t frame_function, const uint8_t *frame_data)
{
    if (frame_function == IMU_FUNC_RAW_ACCEL) {
        float accel_ratio = 16.0f / 32767.0f;
        handle->ax = to_int16(&frame_data[0])  * accel_ratio;
        handle->ay = to_int16(&frame_data[2])  * accel_ratio;
        handle->az = to_int16(&frame_data[4])  * accel_ratio;

        float deg_to_rad = 3.14159265358979323846f / 180.0f;
        float gyro_ratio  = (2000.0f / 32767.0f) * deg_to_rad;
        handle->gx = to_int16(&frame_data[6])  * gyro_ratio;
        handle->gy = to_int16(&frame_data[8])  * gyro_ratio;
        handle->gz = to_int16(&frame_data[10]) * gyro_ratio;

        float mag_ratio = 800.0f / 32767.0f;
        handle->mx = to_int16(&frame_data[12]) * mag_ratio;
        handle->my = to_int16(&frame_data[14]) * mag_ratio;
        handle->mz = to_int16(&frame_data[16]) * mag_ratio;
    } else if (frame_function == IMU_FUNC_EULER) {
        handle->roll  = to_float(&frame_data[0]);
        handle->pitch = to_float(&frame_data[4]);
        handle->yaw   = to_float(&frame_data[8]);
    } else if (frame_function == IMU_FUNC_QUAT) {
        handle->q0 = to_float(&frame_data[0]);
        handle->q1 = to_float(&frame_data[4]);
        handle->q2 = to_float(&frame_data[8]);
        handle->q3 = to_float(&frame_data[12]);
    } else if (frame_function == IMU_FUNC_BARO) {
        handle->height            = to_float(&frame_data[0]);
        handle->temperature       = to_float(&frame_data[4]);
        handle->pressure          = to_float(&frame_data[8]);
        handle->pressure_contrast = to_float(&frame_data[12]);
    } else if (frame_function == IMU_FUNC_VERSION) {
        handle->version_high = frame_data[0];
        handle->version_mid  = frame_data[1];
        handle->version_low  = frame_data[2];
    } else if (frame_function == IMU_FUNC_RETURN_STATE) {
        handle->last_rx_function = frame_data[0];
        handle->last_rx_state    = (int16_t)frame_data[1];
    }
}

/* ---------------- 初始化接口 ---------------- */
void IMU_UART_Init(ImuInstance id)
{
    if (id >= IMU_INSTANCE_MAX) return;
    
    ImuHandle* handle = &imu_handles[id];
    const ImuHwConfig* hw = &hw_configs[id];
    
    /* 关联硬件配置 */
    handle->hw_config = hw;
    
    /* 初始化句柄内的状态变量 */
    handle->rx_write_index = 0;
    handle->rx_read_index = 0;
    handle->rx_state = 0;
    handle->frame_length = 0;
    handle->frame_function = 0;
    handle->frame_index = 0;
    handle->version_high = -1;
    handle->imu_cache_index = 0;
    handle->imu_cache_full = 0;
    IMU_UART_ClearAutoReportData(id);

    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 打开 GPIO 时钟 */
    RCC_APB2PeriphClockCmd(hw->rcc_apb2_gpio, ENABLE);

    /* 打开 USART 时钟 (UART2/3 在 APB1) */
    RCC_APB1PeriphClockCmd(hw->rcc_apb1_usart, ENABLE);

    /* 配置 TX 引脚为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = hw->tx_gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(hw->tx_gpio_port, &GPIO_InitStructure);

    /* 配置 RX 引脚为浮空输入模式 */
    GPIO_InitStructure.GPIO_Pin = hw->rx_gpio_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(hw->rx_gpio_port, &GPIO_InitStructure);

    /* NVIC 配置 */
    NVIC_InitStructure.NVIC_IRQChannel = hw->irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = hw->irq_preemption_priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = hw->irq_sub_priority;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* USART 配置 */
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(hw->usart, &USART_InitStructure);

    /* 开启接收中断 */
    USART_ITConfig(hw->usart, USART_IT_RXNE, ENABLE);
    USART_Cmd(hw->usart, ENABLE);
}

/* ---------------- 中断接收入口 ---------------- */
void IMU_UART_RxBytes(ImuInstance id, volatile uint8_t *data, uint16_t len)
{
    if (id >= IMU_INSTANCE_MAX || !data || len == 0) return;
    ImuHandle* handle = &imu_handles[id];
    
    for (uint16_t i = 0; i < len; ++i) {
        _rxbuf_push(handle, data[i]);
    }
}

/* ---------------- 解析环形缓冲 ---------------- */
void IMU_UART_Process(ImuInstance id)
{
    if (id >= IMU_INSTANCE_MAX) return;
    ImuHandle* handle = &imu_handles[id];

    enum {
        RX_STATE_EXPECT_HEAD1 = 0,
        RX_STATE_EXPECT_HEAD2,
        RX_STATE_EXPECT_LENGTH,
        RX_STATE_EXPECT_FUNCTION,
        RX_STATE_COLLECT_DATA
    };

    uint8_t current_byte = 0;

    while (_rxbuf_pop(handle, &current_byte) == 0) {
        switch (handle->rx_state) {
        case RX_STATE_EXPECT_HEAD1:
            handle->rx_state = (current_byte == FRAME_HEAD1) ? RX_STATE_EXPECT_HEAD2 : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_HEAD2:
            handle->rx_state = (current_byte == FRAME_HEAD2) ? RX_STATE_EXPECT_LENGTH : RX_STATE_EXPECT_HEAD1;
            break;

        case RX_STATE_EXPECT_LENGTH:
            handle->frame_length = current_byte;
            handle->rx_state = RX_STATE_EXPECT_FUNCTION;
            break;

        case RX_STATE_EXPECT_FUNCTION:
            handle->frame_function = current_byte;
            handle->frame_index = 0;
            handle->rx_state = RX_STATE_COLLECT_DATA;
            break;

        case RX_STATE_COLLECT_DATA: {
            uint16_t data_length = (handle->frame_length >= 4) ? (uint16_t)(handle->frame_length - 4) : 0;
            if (data_length == 0 || data_length > sizeof(handle->frame_buffer)) {
                handle->rx_state = RX_STATE_EXPECT_HEAD1;
                break;
            }

            handle->frame_buffer[handle->frame_index++] = current_byte;
            if (handle->frame_index >= data_length) {
                uint8_t calculated_checksum = (uint8_t)(FRAME_HEAD1 + FRAME_HEAD2 + handle->frame_length + handle->frame_function);
                for (uint16_t i = 0; i < data_length - 1; ++i) {
                    calculated_checksum = (uint8_t)(calculated_checksum + handle->frame_buffer[i]);
                }

                uint8_t received_checksum = handle->frame_buffer[data_length - 1];
                if (calculated_checksum == received_checksum) {
                    _parse_frame_data(handle, handle->frame_function, handle->frame_buffer);
                }
                handle->rx_state = RX_STATE_EXPECT_HEAD1;
            }
        } break;

        default:
            handle->rx_state = RX_STATE_EXPECT_HEAD1;
            break;
        }
    }
}

/* ---------------- 帧发送接口 ---------------- */
int IMU_UART_SendCommand(ImuInstance id, uint8_t function, const uint8_t *params, uint8_t param_len)
{
    if (id >= IMU_INSTANCE_MAX) return -1;
    if (param_len > 3 || (param_len > 0 && params == NULL)) {
        return -1;
    }

    uint8_t frame[8] = {FRAME_HEAD1, FRAME_HEAD2, 0, function, 0, 0, 0, 0};

    for (uint8_t i = 0; i < param_len; ++i) {
        frame[4 + i] = params[i];
    }

    uint8_t frame_len = (uint8_t)(4 + param_len + 1);
    frame[2] = frame_len;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < frame_len - 1; ++i) {
        checksum = (uint8_t)(checksum + frame[i]);
    }
    frame[frame_len - 1] = checksum;

    Send_IMU_Array(id, frame, frame_len);
    return 0;
}

/* ---------------- 发送函数 ---------------- */
void Send_IMU_Data(ImuInstance id, uint8_t Data)
{
    if (id >= IMU_INSTANCE_MAX) return;
    const ImuHwConfig* hw = imu_handles[id].hw_config;
    
    while (USART_GetFlagStatus(hw->usart, USART_FLAG_TXE) == RESET);
    USART_SendData(hw->usart, Data);
}

void Send_IMU_Array(ImuInstance id, uint8_t *pData, uint8_t Length)
{
    if (id >= IMU_INSTANCE_MAX) return;
    while (Length--) {
        Send_IMU_Data(id, *pData);
        pData++;
    }
}

/* ---------------- 数据读取接口 ---------------- */
int IMU_UART_GetAccelerometer(ImuInstance id, float out[3])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    out[0] = handle->ax; out[1] = handle->ay; out[2] = handle->az;
    return 0;
}

int IMU_UART_GetGyroscope(ImuInstance id, float out[3])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    out[0] = handle->gx; out[1] = handle->gy; out[2] = handle->gz;
    return 0;
}

int IMU_UART_GetMagnetometer(ImuInstance id, float out[3])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    out[0] = handle->mx; out[1] = handle->my; out[2] = handle->mz;
    return 0;
}

int IMU_UART_GetQuaternion(ImuInstance id, float out[4])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    out[0] = handle->q0; out[1] = handle->q1; out[2] = handle->q2; out[3] = handle->q3;
    return 0;
}

int IMU_UART_GetEuler(ImuInstance id, float out[3])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    const float RAD2DEG = 57.2957795f;
    out[0] = handle->roll  * RAD2DEG;
    out[1] = handle->pitch * RAD2DEG;
    out[2] = handle->yaw   * RAD2DEG;
    return 0;
}

int IMU_UART_GetBarometer(ImuInstance id, float out[4])
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    ImuHandle* handle = &imu_handles[id];
    out[0] = handle->height; out[1] = handle->temperature; out[2] = handle->pressure; out[3] = handle->pressure_contrast;
    return 0;
}

void IMU_UART_GetVersion(ImuInstance id)
{
    if (id >= IMU_INSTANCE_MAX) return;
    ImuHandle* handle = &imu_handles[id];
    
    if (handle->version_high < 0) {
        uint8_t payload[2] = {IMU_FUNC_VERSION, 0x00};
        IMU_UART_SendCommand(id, IMU_FUNC_REQUEST_DATA, payload, (uint8_t)sizeof(payload));

        for (int i = 0; i < 20; ++i) {
            IMU_UART_Process(id);
            if (handle->version_high >= 0) {
                printf("IMU%d Version:%d.%d.%d\r\n", id+1, handle->version_high, handle->version_mid, handle->version_low);
                return;
            }
            delay_ms(5);
        }
        printf("IMU%d Version:-1\r\n", id+1);
        return;
    }
}

int IMU_UART_GetAll(ImuInstance id, imu_measurement_t *out)
{
    if (id >= IMU_INSTANCE_MAX || !out) return -1;
    IMU_UART_GetAccelerometer(id, out->accel);
    IMU_UART_GetGyroscope(id, out->gyro);
    IMU_UART_GetMagnetometer(id, out->mag);
    IMU_UART_GetQuaternion(id, out->quat);
    IMU_UART_GetEuler(id, out->euler);
    IMU_UART_GetBarometer(id, out->baro);
    return 0;
}

/* ---------------- 清理缓存 ---------------- */
void IMU_UART_ClearAutoReportData(ImuInstance id)
{
    if (id >= IMU_INSTANCE_MAX) return;
    ImuHandle* handle = &imu_handles[id];
    handle->ax = handle->ay = handle->az = 0.0f;
    handle->gx = handle->gy = handle->gz = 0.0f;
    handle->mx = handle->my = handle->mz = 0.0f;
    handle->roll = handle->pitch = handle->yaw = 0.0f;
    handle->q0 = handle->q1 = handle->q2 = handle->q3 = 0.0f;
    handle->height = handle->temperature = handle->pressure = handle->pressure_contrast = 0.0f;
}

/* ---------------- 校准相关 (内部辅助函数) ---------------- */
static int _calibration_with_wait(ImuInstance id, uint8_t function, const uint8_t *payload, uint8_t payload_len,
                                  const char *label, uint32_t timeout_ms)
{
    if (id >= IMU_INSTANCE_MAX) return -1;
    ImuHandle* handle = &imu_handles[id];
    
    handle->last_rx_function = 0;
    handle->last_rx_state = -1;

    int rc = IMU_UART_SendCommand(id, function, payload, payload_len);
    if (rc != 0) {
        return rc;
    }

    int result = IMU_UART_WaitCalibration(id, function, timeout_ms);
    if (!label) {
        label = "unknown";
    }

    if (result == -1) {
        printf("[IMU%d] Calibration %s timeout\r\n", id+1, label);
    } else if (result == 1) {
        printf("[IMU%d] Calibration %s success\r\n", id+1, label);
    } else {
        printf("[IMU%d] Calibration %s failed (code=%d)\r\n", id+1, label, result);
    }

    return result;
}

int IMU_UART_CalibrationImu(ImuInstance id)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return _calibration_with_wait(id, IMU_FUNC_CALIB_IMU, payload, (uint8_t)sizeof(payload), "imu", 7000);
}

int IMU_UART_CalibrationMag(ImuInstance id)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return _calibration_with_wait(id, IMU_FUNC_CALIB_MAG, payload, (uint8_t)sizeof(payload), "mag", 0);
}

int IMU_UART_CalibrationTemp(ImuInstance id, float now_temperature)
{
    if (id >= IMU_INSTANCE_MAX) return -1;
    if (now_temperature > 50.0f || now_temperature < -50.0f) {
        return -1;
    }
    int16_t temperature_raw = (int16_t)(now_temperature * 100.0f);
    uint8_t param_low  = (uint8_t)(temperature_raw & 0xFF);
    uint8_t param_high = (uint8_t)((temperature_raw >> 8) & 0xFF);
    uint8_t payload[3] = {param_low, param_high, 0x5F};
    return _calibration_with_wait(id, IMU_FUNC_CALIB_TEMP, payload, (uint8_t)sizeof(payload), "temp", 2000);
}

int IMU_UART_ResetUserData(ImuInstance id)
{
    uint8_t payload[2] = {0x01, 0x5F};
    return IMU_UART_SendCommand(id, IMU_FUNC_RESET_FLASH, payload, (uint8_t)sizeof(payload));
}

int IMU_UART_WaitCalibration(ImuInstance id, uint8_t function, uint32_t timeout_ms)
{
    if (id >= IMU_INSTANCE_MAX) return -1;
    ImuHandle* handle = &imu_handles[id];
    
    uint32_t elapsed_ms = 0;
    while (1) {
        IMU_UART_Process(id);

        if (handle->last_rx_function == function) {
            return handle->last_rx_state;
        }

        if (timeout_ms != 0 && elapsed_ms >= timeout_ms) {
            return -1;
        }

        delay_ms(1);
        if (timeout_ms != 0) {
            ++elapsed_ms;
        }
    }
}

/* ---------------- IMU 缓存操作 ---------------- */
void IMU_Buffer_Push(ImuInstance id)
{
    if (id >= IMU_INSTANCE_MAX) return;
    ImuHandle* handle = &imu_handles[id];
    
    handle->imu_cache[handle->imu_cache_index].roll  = handle->roll;
    handle->imu_cache[handle->imu_cache_index].pitch = handle->pitch;
    handle->imu_cache[handle->imu_cache_index].yaw   = handle->yaw;

    handle->imu_cache_index++;

    if(handle->imu_cache_index >= IMU_CACHE_SIZE)
    {
        handle->imu_cache_index = 0;
        handle->imu_cache_full = 1;
    }
}

/* ---------------- 中断处理函数 ---------------- */

/* USART2 中断 (对应 IMU_INSTANCE_1) */
void USART2_IRQHandler(void)
{
    uint8_t rx_byte;

    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        rx_byte = USART_ReceiveData(USART2);
        IMU_UART_RxBytes(IMU_INSTANCE_1, &rx_byte, 1);
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

/* USART3 中断 (对应 IMU_INSTANCE_2) */
void USART3_IRQHandler(void)
{
    uint8_t rx_byte;

    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        rx_byte = USART_ReceiveData(USART3);
        IMU_UART_RxBytes(IMU_INSTANCE_2, &rx_byte, 1);
        USART_ClearITPendingBit(USART3, USART_IT_RXNE);
    }
}
