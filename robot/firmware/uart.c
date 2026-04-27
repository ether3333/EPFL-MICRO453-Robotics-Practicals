/******************************************************************************
 *
 * This module provides interface routines to the LPC ARM UARTs.
 * Copyright 2004, R O SoftWare
 * No guarantees, warrantees, or promises, implied or otherwise.
 * May be used for hobby or commercial purposes provided copyright
 * notice remains intact.
 *
 *****************************************************************************/
#include <limits.h>
#include <stdint.h>
#include "LPC21xx.h"
#include "uart.h"

#if UART0_SUPPORT

/******************************************************************************
 *
 * Function Name: uart0Init()
 *
 * Description:
 *    This function initializes the UART for async mode
 *
 * Calling Sequence:
 *    baudrate divisor - use UART_BAUD macro
 *    mode - see typical modes (uart.h)
 *    fmode - see typical fmodes (uart.h)
 *
 * Returns:
 *    void
 *
 * NOTE: uart0Init(UART_BAUD(9600), UART_8N1, UART_FIFO_8);
 *
 *****************************************************************************/
void uart0Init(uint16_t baud, uint8_t mode, uint8_t fmode)
{
  // set port pins for UART0
  PINSEL0 = (PINSEL0 & ~U0_PINMASK) | U0_PINSEL;

  U0IER = 0x00;                         // disable all interrupts
  U0IIR;                                // clear interrupt ID
  U0RBR;                                // clear receive register
  U0LSR;                                // clear line status register

  // set the baudrate
  U0LCR = ULCR_DLAB_ENABLE;             // select divisor latches
  U0DLL = (uint8_t)baud;                // set for baud low byte
  U0DLM = (uint8_t)(baud >> 8);         // set for baud high byte

  // set the number of characters and other
  // user specified operating parameters
  U0LCR = (mode & ~ULCR_DLAB_ENABLE);
  U0FCR = fmode;

#if defined(UART0_TX_INT_MODE) || defined(UART0_RX_INT_MODE)
  // initialize the interrupt vector
  VICIntSelect &= ~VIC_BIT(VIC_UART0);  // UART0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART0);    // UART0 interrupt enabled
  VICVectCntl0 = VIC_ENABLE | VIC_UART0;
  VICVectAddr0 = (uint32_t)uart0ISR;    // address of the ISR

#ifdef UART0_TX_INT_MODE
  // initialize the transmit data queue
  uart0_tx_extract_idx = uart0_tx_insert_idx = 0;
  uart0_tx_running = 0;
#endif

#ifdef UART0_RX_INT_MODE
  // initialize the receive data queue
  uart0_rx_extract_idx = uart0_rx_insert_idx = 0;

  // enable receiver interrupts
  U0IER = UIER_ERBFI;
#endif
#endif
}

/******************************************************************************
 *
 * Function Name: uart0Putch()
 *
 * Description:
 *    This function puts a character into the UART output queue for
 *    transmission.
 *
 * Calling Sequence:
 *    character to be transmitted
 *
 * Returns:
 *    ch on success, -1 on error (queue full)
 *
 *****************************************************************************/
int uart0Putch(int ch)
{
#ifdef UART0_TX_INT_MODE
  uint16_t temp;
  unsigned cpsr;

  temp = (uart0_tx_insert_idx + 1) % UART0_TX_BUFFER_SIZE;

  if (temp == uart0_tx_extract_idx)
    return -1;                          // no room

  cpsr = disableIRQ();                  // disable global interrupts
  U0IER &= ~UIER_ETBEI;                 // disable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts

  // check if in process of sending data
  if (uart0_tx_running)
    {
    // add to queue
    uart0_tx_buffer[uart0_tx_insert_idx] = (uint8_t)ch;
    uart0_tx_insert_idx = temp;
    }
  else
    {
    // set running flag and write to output register
    uart0_tx_running = 1;
    U0THR = (uint8_t)ch;
    }

  cpsr = disableIRQ();                  // disable global interrupts
  U0IER |= UIER_ETBEI;                  // enable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts
#else
  while (!(U0LSR & ULSR_THRE))          // wait for TX buffer to empty
    continue;                           // also either WDOG() or swap()

  U0THR = (uint8_t)ch;
#endif
  return (uint8_t)ch;
}

/******************************************************************************
 *
 * Function Name: uart0Space()
 *
 * Description:
 *    This function gets the available space in the transmit queue
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    available space in the transmit queue
 *
 *****************************************************************************/
uint16_t uart0Space(void)
{
#ifdef UART0_TX_INT_MODE
  int space;

  if ((space = (uart0_tx_extract_idx - uart0_tx_insert_idx)) <= 0)
    space += UART0_TX_BUFFER_SIZE;

  return (uint16_t)(space - 1);
#else
  return USHRT_MAX;
#endif
}

/******************************************************************************
 *
 * Function Name: uart0Puts()
 *
 * Description:
 *    This function writes a NULL terminated 'string' to the UART output
 *    queue, returning a pointer to the next character to be written.
 *
 * Calling Sequence:
 *    address of the string
 *
 * Returns:
 *    a pointer to the next character to be written
 *    (\0 if full string is written)
 *
 *****************************************************************************/
const char *uart0Puts(const char *string)
{
  register char ch;

  while ((ch = *string) && (uart0Putch(ch) >= 0))
    string++;

  return string;
}

/******************************************************************************
 *
 * Function Name: uart0Write()
 *
 * Description:
 *    This function writes 'count' characters from 'buffer' to the UART
 *    output queue.
 *
 * Calling Sequence:
 *
 *
 * Returns:
 *    0 on success, -1 if insufficient room, -2 on error
 *    NOTE: if insufficient room, no characters are written.
 *
 *****************************************************************************/
int uart0Write(const char *buffer, uint16_t count)
{
#ifdef UART0_TX_INT_MODE
  if (count > uart0Space())
    return -1;
#endif
  while (count && (uart0Putch(*buffer++) >= 0))
    count--;

  return (count ? -2 : 0);
}

/******************************************************************************
 *
 * Function Name: uart0TxEmpty()
 *
 * Description:
 *    This function returns the status of the UART transmit data
 *    registers.
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    FALSE - either the tx holding or shift register is not empty
 *   !FALSE - if both the tx holding & shift registers are empty
 *
 *****************************************************************************/
int uart0TxEmpty(void)
{
  return (U0LSR & (ULSR_THRE | ULSR_TEMT)) == (ULSR_THRE | ULSR_TEMT);
}

/******************************************************************************
 *
 * Function Name: uart0TxFlush()
 *
 * Description:
 *    This function removes all characters from the UART transmit queue
 *    (without transmitting them).
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    void
 *
 *****************************************************************************/
void uart0TxFlush(void)
{
#ifdef UART0_TX_INT_MODE
  unsigned cpsr;

  U0FCR |= UFCR_TX_FIFO_RESET;          // clear the TX fifo

  // "Empty" the transmit buffer.
  cpsr = disableIRQ();                  // disable global interrupts
  U0IER &= ~UIER_ETBEI;                 // disable TX interrupts
  restoreIRQ(cpsr);                     // restore global interrupts
  uart0_tx_insert_idx = uart0_tx_extract_idx = 0;
#else
  U0FCR |= UFCR_TX_FIFO_RESET;          // clear the TX fifo
#endif
}

/******************************************************************************
 *
 * Function Name: uart0Getch()
 *
 * Description:
 *    This function gets a character from the UART receive queue
 *
 * Calling Sequence:
 *    void
 *
 * Returns:
 *    character on success, -1 if no character is available
 *
 *****************************************************************************/
int uart0Getch(void)
{
#ifdef UART0_RX_INT_MODE
  uint8_t ch;

  if (uart0_rx_insert_idx == uart0_rx_extract_idx) // check if character is available
    return -1;

  ch = uart0_rx_buffer[uart0_rx_extract_idx++]; // get character, bump pointer
  uart0_rx_extract_idx %= UART0_RX_BUFFER_SIZE; // limit the pointer
  return ch;
#else
  if (U0LSR & ULSR_RDR)                 // check if character is available
    return U0RBR;                       // return character

  return -1;
#endif
}

#endif

#ifdef UART0_RX_INT_MODE
#error RX Interrupt mode has not to be there...
#endif

#ifdef UART0_TX_INT_MODE
#error TX Interrupt mode has not to be there...
#endif
