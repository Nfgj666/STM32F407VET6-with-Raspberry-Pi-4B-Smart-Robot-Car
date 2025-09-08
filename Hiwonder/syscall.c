/**
 * @file syscall.c
 * @author Lu Yongping (Lucas@hiwonder.com)
 * @brief 标准库桩函数的重定向
 * @version 0.1
 * @date 2023-05-12
 *
 * @copyright Copyright (c) 2023
 *
 */


#include <stdio.h>
#include "usart.h"

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE* f)
#endif /* __GNUC__ */

#pragma import(__use_no_semihosting) /* 以下代码不适用半主机模式 */

void _sys_exit(int x)
{
}

struct __FILE {
    int handle;
};
FILE __stdout;


PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, 0xFFFF); /* 重定向到 UART1 打印 */
    // SEGGER_RTT_Write(0, &ch, 1);   /* 重定向到 JLINK RTT 打印, 使用RTT时需要先初始化RTT */
    return (ch);
}


