/******************************************************************************
 * Routines for accessing the I2C bus on the LPC2129 microcontroller.
 * Alessandro Crespi, Mar. 2009
 *****************************************************************************/

#include "i2c.h"
#include "hwconfig.h"
#include "LPC21xx.h"
#include "armVIC.h"
#include "timerISR.h"

void i2c_init()
{
  uint32_t i;

  PINSEL0 &= 0xFFFFFF0F;         // select SCL & SDA functions
  PINSEL0 |= 0x00000050;         // for the concerned pins
  i = PCLK / (2 * 100000);       // I2C rate = 100 kbps
  I2SCLH = i;                    // i = 150 for 60 MHz clock
  I2SCLL = i;
  i2c_reset();
}

void i2c_reset()
{
  I2CONCLR = 0x6C;               // clear all
  I2CONSET = 0x40;               // enable the I2C module
}

uint8_t i2c_set(uint8_t addr, uint8_t reg, uint8_t val)
{
  unsigned irqs = disable_timer1_irq();  // save T1 interrupt status & ensure disabled (T1 ISR uses I2C sometimes...)
  I2CONCLR = 0x2C;               // clear STA, SI & AA
  I2CONSET = 0x20;               // send start condition
  while (I2STAT == 0xF8);        // wait for complete start condition
  if (I2STAT != 0x08) {          // start error
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0;
  }
  I2DAT = (addr << 1);           // send destination address + W/
  I2CONCLR = 0x28;               // clear STA & SI
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x18) {          // no ACK or other error
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0;
  }
  I2DAT = reg;                   // send destination register
  I2CONCLR = 0x0C;               // clear SI & AA
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x28) {          // return on error or no ACK
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0;
  }
  I2DAT = val;                   // send data
  I2CONCLR = 0x0C;               // clear SI & AA
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x28) {          // return on error or no ACK
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0;
  }
  I2CONSET = 0x10;               // send stop condition
  I2CONCLR = 0x0C;               // clear SI & AA
  restore_timer1_irq(irqs);      // enable T1 interrupts if active before
  return 1;
}

uint8_t i2c_get(uint8_t addr, uint8_t reg)
{
  uint8_t res = 0xFF;
  unsigned irqs = disable_timer1_irq();  // save T1 interrupt status & ensure disabled
  I2CONCLR = 0x2C;               // clear STA, SI & AA
  I2CONSET = 0x20;               // send start condition
  while (I2STAT == 0xF8);        // wait for complete start condition
  if (I2STAT != 0x08) {          // start error
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  I2DAT = (addr << 1);           // send destination address + W/
  I2CONCLR = 0x28;               // clear STA & SI
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x18) {          // no ACK or other error
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  I2DAT = reg;                   // send destination register
  I2CONCLR = 0x0C;               // clear SI & AA
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x28) {          // return on error or no ACK
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  I2CONCLR = 0x0C;               // clear SI & AA
  I2CONSET = 0x20;               // send repeated start condition
  while (I2STAT == 0xF8);        // wait for complete RS
  if (I2STAT != 0x10) {          // repeated start error
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  I2DAT = (addr << 1) | 1;       // send destination address + R
  I2CONCLR = 0x28;               // clear STA & SI
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x40) {          // return on error or no ACK
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  I2CONCLR = 0x0C;               // clear SI & AA
  while (I2STAT == 0xF8);        // wait for status change
  if (I2STAT != 0x58) {          // return if !(data received, no ACK)
    i2c_reset();
    restore_timer1_irq(irqs);
    return 0xFF;
  }
  res = I2DAT;                   // read incoming data
  I2CONSET = 0x10;               // send stop condition
  I2CONCLR = 0x0C;               // clear SI & AA
  restore_timer1_irq(irqs);      // restore T1 interrupt status
  return res;
}
