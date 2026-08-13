#include "usart.h"	 
/***


这个部分仅仅用作串口的重定向，所以模块用的串口初始化全部移动到了对应的模块里面
***/

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	
//Add the following code to support the printf function without selecting use MicroLIB
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数   Support functions required by the standard library
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式  Define _sys_exit() to avoid using semihosting mode
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 Redefine fputc function
int fputc(int ch, FILE *f)
{
    while((USART1->SR & USART_SR_TXE) == 0);

    USART1->DR = (uint8_t)ch;

    return ch;
}
#endif 

/**
 * @Brief: UART1发送数据		UART1 sends data
 * @Note: 
 * @Parm: ch:待发送的数据 	ch: data to be sent
 * @Retval: 
 */
void USART1_Send_U8(uint8_t ch)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
	USART_SendData(USART1, ch);
}

/**
 * @Brief: UART1发送数据		UART1 sends data
 * @Note: 
 * @Parm: BufferPtr:待发送的数据  Length:数据长度		BufferPtr: data to be sent Length: data length
 * @Retval: 
 */
void USART1_Send_ArrayU8(uint8_t *BufferPtr, uint16_t Length)
{
	while (Length--)
	{
		USART1_Send_U8(*BufferPtr);
		BufferPtr++;
	}
}


