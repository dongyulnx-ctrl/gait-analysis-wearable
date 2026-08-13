#include "emg_feature.h"

/* 外部声明EMG原始值变量（来自emg_uart.c） */
extern uint16_t emg_ch1_raw;
extern uint16_t emg_ch2_raw;

void EMG_Raw_Init(void)
{
    /* 空函数，保留接口兼容性 */
}

void EMG_Raw_Get(uint16_t *emg1_raw, uint16_t *emg2_raw)
{
    /* 检查指针是否为空 */
    if(!emg1_raw || !emg2_raw) return;

    /* 直接返回原始值 */
    *emg1_raw = emg_ch1_raw;
    *emg2_raw = emg_ch2_raw;
}