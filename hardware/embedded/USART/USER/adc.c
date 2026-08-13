#include "adc.h"

void Adc_GPIO_Config(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* 使能GPIO和ADC1时钟 */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);
  
  /* 配置PA4和PA5为模拟输入 */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Adc_Config(void)
{ 
  ADC_InitTypeDef ADC_InitStructure; 
  Adc_GPIO_Config();
  
  /* ADC时钟配置 */
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);
  
  /* 重置ADC1寄存器 */
  ADC_DeInit(ADC1);
  
  /* ADC工作模式配置 - 关键修改：单通道模式，单次转换 */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;    // 关闭扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 关闭连续转换
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;          // 通道数量为1
  
  ADC_Init(ADC1, &ADC_InitStructure);
  
  /* 使能ADC1 */
  ADC_Cmd(ADC1, ENABLE);
  
  /* 校准ADC */
  ADC_ResetCalibration(ADC1);
  while(ADC_GetResetCalibrationStatus(ADC1));
  ADC_StartCalibration(ADC1);
  while(ADC_GetCalibrationStatus(ADC1));
}

/* 获取两个ADC通道的值（软件轮询方式） */
void Get_Two_ADC_Values(uint16_t *value1, uint16_t *value2)
{
  // --- 读取传感器1 (通道4, PA4) ---
  // 配置通道4，采样时间设为较长以保证精度
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_239Cycles5);
  
  // 启动转换
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  // 等待转换结束
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  
  // 读取结果
  *value1 = ADC_GetConversionValue(ADC1);

  
  // --- 读取传感器2 (通道5, PA5) ---
  // 配置通道5
  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_239Cycles5);
  
  // 启动转换
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  // 等待转换结束
  while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
  
  // 读取结果
  *value2 = ADC_GetConversionValue(ADC1);
}
