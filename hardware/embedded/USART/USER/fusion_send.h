#ifndef __FUSION_SEND_H
#define __FUSION_SEND_H

#include "stdint.h"

/* 修改：添加两个 uint16_t 的 ADC 参数 */
void Fusion_Send_Update(uint16_t adc1, uint16_t adc2);

#endif /* __FUSION_SEND_H */