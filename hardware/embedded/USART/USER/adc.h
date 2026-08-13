#ifndef __ADC_H
#define __ADC_H	
#include "stm32f10x.h"

void Adc_GPIO_Config(void); 
void Adc_Config(void);

/* 添加这一行：声明 Get_Two_ADC_Values 函数 */
void Get_Two_ADC_Values(uint16_t *value1, uint16_t *value2);

#endif /* __ADC_H */