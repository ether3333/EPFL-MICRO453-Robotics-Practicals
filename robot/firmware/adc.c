#include "adc.h"
#include "LPC21xx.h"
#include "lpcADC.h"
#include "uart.h"
#include "timerISR.h"
#include "sysTime.h"

void adc_init()
{
  PINSEL1 &= ~(0xFF << 22);    // Set AN0...AN3 as analog inputs
  PINSEL1 |= (0x55 << 22);
  // clock = (30 / (6+1)) = 4.29 MHz, A/D enabled, pin Ain0
  ADCR = (6 << 8) | (1 << 21) | (1 << 0);
}

void adc_set_channel(uint8_t ch)
{
  if (ch>3) {
    ch = 0;
  }
  ADCR = (ADCR & 0xFFFFFF00) | (1 << ch);
}

uint16_t adc_read()
{
  ADCR |= (1 << 24);               // start now
  while (!(ADDR & 0x80000000));    // wait for conversion
  ADCR &= ~(1 << 24);              // stop conversion (important!)
  return (ADDR & 0x0000FFFF) >> 6;
}

uint16_t adc_read_ch(uint8_t ch)
{
  unsigned irqs = disable_timer1_irq();  // save T1 interrupt status & ensure disabled
  uint16_t res;

  adc_set_channel(ch);
  pause(100);
  res = adc_read();
  restore_timer1_irq(irqs);
  return res;
}
