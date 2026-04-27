#ifndef __PRINTF_H
#define __PRINTF_H

#include "uart.h"

int uart_printf(const char *format, ...);
int sprintf(char *out, const char *format, ...);
void set_printf_uart(uint8_t u);

#endif // __PRINTF_H
