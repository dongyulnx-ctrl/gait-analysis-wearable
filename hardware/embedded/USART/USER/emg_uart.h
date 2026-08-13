#ifndef __EMG_UART_H
#define __EMG_UART_H

#include "stdint.h"

extern uint16_t emg_ch1_raw;
extern uint16_t emg_ch1_rms;

extern uint16_t emg_ch2_raw;
extern uint16_t emg_ch2_rms;

void EMG_UART_Init(void);
void EMG_UART_Process(void);
void EMG_UART_RxByte(uint8_t data);

#endif
