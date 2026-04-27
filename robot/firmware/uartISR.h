#ifndef __UART_ISR_H
#define __UART_ISR_H

#include <stdint.h>

// Initializes the ISR for the UART0
void init_uart0_isr(void);

// Interrupt Service Routine for UART0 (i.e. wireless controller PIC)
void uart0ISR(void) __attribute__((naked));

// Reads a character from UART0, waiting if nothing is available
uint8_t uart0_waitch(void);

#endif
