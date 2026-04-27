/******************************************************************************
 * radio.c - Routines for handling the radio communication protocol
 * Alessandro Crespi & Jeremie Knuesel, Sep. 2010, updated March 2013
 *****************************************************************************/

#include <stdint.h>
#include "radio.h"
#include "registers.h"
#include "uart.h"
#include "uartISR.h"
#include "utils.h"

// Register banks
volatile uint8_t reg8_table[REG8_MAX];       ///< 8-bit register bank
volatile uint16_t reg16_table[REG16_MAX];    ///< 16-bit register bank
volatile uint32_t reg32_table[REG32_MAX];    ///< 32-bit register bank

#define MAX_REG_CALLBACKS 16

// Callback function addresses (NULL if entry unused)
static register_callback_t reg_callbacks[MAX_REG_CALLBACKS];
static RadioData regdata2;

void radio_add_reg_callback(register_callback_t addr)
{
  uint8_t i;

  for (i = 0; i < MAX_REG_CALLBACKS; i++) {
    if (reg_callbacks[i] == NULL) {
      reg_callbacks[i] = addr;
      break;
    }
  }
}

void radio_del_reg_callback(register_callback_t addr)
{
  uint8_t i;

  for (i = 0; i < MAX_REG_CALLBACKS; i++) {
    if (reg_callbacks[i] == addr) {
      reg_callbacks[i] = NULL;
      break;
    }
  }
}

static void callbacks_call_all(const uint8_t operation, const uint8_t address)
{
  uint8_t i;

  for (i = 0; i < MAX_REG_CALLBACKS; i++) {
    if (reg_callbacks[i] != NULL) {
      reg_callbacks[i](operation, address, &regdata2);
    }
  }
}

static uint8_t callbacks_call_one(const uint8_t operation, const uint8_t address)
{
  uint8_t i;

  for (i = 0; i < MAX_REG_CALLBACKS; i++) {
    if (reg_callbacks[i] != NULL && reg_callbacks[i](operation, address, &regdata2))
    return TRUE;
  }
  return FALSE;
}


static void internal_reg_read_8(uint16_t addr)
{
  if (callbacks_call_one (ROP_READ_8, addr))
    return;

  // If the register address is in the static register table, return the
  // corresponding table contents. If it is not, return 0xFF.
  if (addr < REG8_MAX)
    regdata2.byte = reg8_table[addr];
  else
    regdata2.byte = 0xFF;
}

static void internal_reg_read_16(uint16_t addr)
{
  if (callbacks_call_one (ROP_READ_16, addr))
    return;

  // If the register address is in the static register table, return the
  // corresponding table contents. If it is not, return 0xFFFF.
  if (addr < REG16_MAX)
    regdata2.word = reg16_table[addr];
  else
    regdata2.word = 0xFFFF;
}

static void internal_reg_read_32(uint16_t addr)
{
  if (callbacks_call_one (ROP_READ_32, addr))
    return;

  // If the register address is in the static register table, return the
  // corresponding table contents. If it is not, return 0xFFFFFFFF.
  if (addr < REG32_MAX)
    regdata2.dword = reg32_table[addr];
  else
    regdata2.dword = 0xFFFFFFFF;
}

static void internal_reg_read_mb(uint16_t addr)
{
  regdata2.multibyte.size = 0; // default value
  callbacks_call_one (ROP_READ_MB, addr);
}

static void internal_reg_write_8(uint16_t addr)
{
  // If the register address is valid, write the data to it
  if (addr < REG8_MAX) {
    reg8_table[addr] = regdata2.byte;
  }

  callbacks_call_all (ROP_WRITE_8, addr);
}

static void internal_reg_write_16(uint16_t addr)
{
  // If the register address is valid, write the data to it
  if (addr < REG16_MAX) {
    reg16_table[addr] = regdata2.word;
  }

  callbacks_call_all (ROP_WRITE_16, addr);
}

static void internal_reg_write_32(uint16_t addr)
{
  // If the register address is valid, write the data to it
  if (addr < REG32_MAX) {
    reg32_table[addr] = regdata2.dword;
  }

  callbacks_call_all (ROP_WRITE_32, addr);
}

static void internal_reg_write_mb(uint16_t addr)
{
  callbacks_call_all (ROP_WRITE_MB, addr);
}

void process_UART_in()
{
  uint8_t b1, b2, i;
  uint8_t op, cnt;
  uint16_t addr;
  uint8_t *input_buffer;

  // Reads the first request bytes
  b1 = uart0_waitch();
  b2 = uart0_waitch();

  op = (b1 >> 2);
  addr = ((uint16_t)(b1 & 0x03) << 8) | b2;

  input_buffer = regdata2.bytes;

  // Computes how many bytes we should read
  switch (op) {
    case ROP_WRITE_8:
      cnt = 1;
      break;
    case ROP_WRITE_16:
      cnt = 2;
      break;
    case ROP_WRITE_32:
      cnt = 4;
      break;
    case ROP_WRITE_MB:
      cnt = uart0_waitch();
      input_buffer = regdata2.multibyte.data;
      regdata2.multibyte.size = cnt;
      break;
    default:
      cnt = 0;
  }

  // Reads the request bytes
  for (i = 0; i < cnt; i++)
    input_buffer[i] = uart0_waitch();

  // Calls the appropriate function
  switch (op) {
    case ROP_READ_8:  // byte read
      internal_reg_read_8(addr);
      cnt = 1;
      break;
    case ROP_READ_16:  // word read
      internal_reg_read_16(addr);
      cnt = 2;
      break;
    case ROP_READ_32:  // dword read
      internal_reg_read_32(addr);
      cnt = 4;
      break;
    case ROP_READ_MB:  // multibyte read
      internal_reg_read_mb(addr);
      cnt = regdata2.multibyte.size + 1;
      break;
    case ROP_WRITE_8:  // byte write
      internal_reg_write_8(addr);
      cnt = 0;
      break;
    case ROP_WRITE_16:  // word write
      internal_reg_write_16(addr);
      cnt = 0;
      break;
    case ROP_WRITE_32:  // dword write
      internal_reg_write_32(addr);
      cnt = 0;
      break;
    case ROP_WRITE_MB:  // multibyte write
      internal_reg_write_mb(addr);
      cnt = 0;
      break;
    default:
      cnt = 0;
  }

  // Sends any output bytes
  for (i = 0; i < cnt; i++) uart0Putch(regdata2.bytes[i]);

}

// UART synchronization with the radio controlling PIC
static void sync_radio_pic(void)
{
  uint8_t i;
  do {
    i = uart0_waitch();
  } while (i!=0xAA);
  uart0Putch(0x55);
}

void radio_init()
{
  uint8_t i;

  for (i = 0; i < MAX_REG_CALLBACKS; i++) {
    reg_callbacks[i] = NULL;
  }
  uart0Init(UART_BAUD(57600), UART_8N1, UART_FIFO_8); // setup the UART
  init_uart0_isr();
  IO0SET = LED1_BIT;   // internal LED on
  sync_radio_pic();
  IO0CLR = LED1_BIT;   // internal LED on
}
