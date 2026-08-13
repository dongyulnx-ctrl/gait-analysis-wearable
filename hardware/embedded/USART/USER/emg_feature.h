#ifndef __EMG_FEATURE_H
#define __EMG_FEATURE_H

#include "stdint.h"

/* 初始化（可选，保留空函数） */
void EMG_Raw_Init(void);

/* 获取双EMG原始值（通过指针返回） */
void EMG_Raw_Get(uint16_t *emg1_raw, uint16_t *emg2_raw);

#endif /* __EMG_FEATURE_H */
