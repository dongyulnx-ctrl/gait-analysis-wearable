#include "stm32f10x.h"
#include "emg_uart.h"

uint8_t emg_buf[16];
uint8_t emg_index = 0;

uint16_t emg_ch1_raw;
uint16_t emg_ch1_rms;

uint16_t emg_ch2_raw;
uint16_t emg_ch2_rms;

/* ---------------- 修改说明：将 UART3 改为 UART1 (PA9 TX, PA10 RX) ---------------- */

void EMG_UART_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 修改点 1：GPIO 时钟改为 GPIOA，UART1 时钟在 APB2 上 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    // 删除原来的 RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    /* 修改点 2：TX 引脚改为 PA9 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;       /* 原来是 GPIO_Pin_10 (GPIOB) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);           /* 原来是 GPIOB */

    /* 修改点 3：RX 引脚改为 PA10 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;      /* 原来是 GPIO_Pin_11 (GPIOB) */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);           /* 原来是 GPIOB */

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

    /* 修改点 4：USART 外设改为 USART1 */
    USART_Init(USART1, &USART_InitStructure);        /* 原来是 USART3 */

    /* 修改点 5：开启 USART1 接收中断 */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  /* 原来是 USART3 */

    /* 修改点 6：使能 USART1 */
    USART_Cmd(USART1, ENABLE);                        /* 原来是 USART3 */

    /* 修改点 7：中断向量改为 USART1_IRQn */
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; /* 原来是 USART3_IRQn */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure);
}

/* 数据接收解析 (保持不变) */
void EMG_UART_RxByte(uint8_t data)
{
    static uint8_t state = 0;

    switch(state)
    {
        case 0: // 等待AA
            if(data == 0xAA)
            {
                emg_buf[0] = data;
                state = 1;
            }
            break;

        case 1: // 等待BB
            if(data == 0xBB)
            {
                emg_buf[1] = data;
                emg_index = 2;
                state = 2;
            }
            else
            {
                state = 0;
            }
            break;

        case 2: // 接收剩余数据
            emg_buf[emg_index++] = data;

            if(emg_index >= 10)
            {
                emg_ch1_raw = emg_buf[2] | (emg_buf[3]<<8);
                emg_ch1_rms = emg_buf[4] | (emg_buf[5]<<8);

                emg_ch2_raw = emg_buf[6] | (emg_buf[7]<<8);
                emg_ch2_rms = emg_buf[8] | (emg_buf[9]<<8);

                state = 0;
            }
            break;
    }
}

/* 数据处理 (保持不变) */
void EMG_UART_Process(void)
{

}

/* 修改点 8：中断服务函数改为 USART1_IRQHandler */
void USART1_IRQHandler(void)  /* 原来是 USART3_IRQHandler */
{
    uint8_t data;

    /* 修改点 9：所有 USART3 改为 USART1 */
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        data = (uint8_t)USART_ReceiveData(USART1);

        EMG_UART_RxByte(data);

        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}
