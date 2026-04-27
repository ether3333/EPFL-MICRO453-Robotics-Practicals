#include <stdint.h>
#include "LPC21xx.h"
#include "timerISR.h"
#include "armVIC.h"
#include "registers.h"

static void timer1ISR(void) __attribute__ ((interrupt));

struct FunctionEntry {
  TimerFunction ptr;
  uint16_t counter;
  uint16_t period;
};

static struct FunctionEntry function_table[MAX_TIMER1_USER_FUNCTIONS];

// Uses IRQ1
void timer1_init_isr()
{
  disable_timer1_irq();
  VICIntSelect &= ~VIC_BIT(VIC_TIMER1);
  VICIntEnable = VIC_BIT(VIC_TIMER1);
  VICVectCntl1 = VIC_ENABLE | VIC_TIMER1;
  VICVectAddr1 = (uint32_t) timer1ISR;
  enable_timer1_irq();
}

void timer1_init(uint32_t match_count)
{
  uint8_t i;

  for (i=0; i<MAX_TIMER1_USER_FUNCTIONS; i++) {
    function_table[i].ptr = 0;
  }
  T1TCR = TCR_RESET;  // timer in reset mode
  T1MR0 = match_count;
  T1MCR = TMCR_MR0_R;
  T1PR = 29;  // 1 microsecond resolution at pclk = 30 MHz
  T1CCR = 0;
  T1EMR = 0;
  T1TCR = TCR_ENABLE;  // start counting
}

static void timer1ISR()
{
  uint8_t l;

  // Call user timer functions
  for (l=0; l<MAX_TIMER1_USER_FUNCTIONS; l++) {
    if (function_table[l].ptr) {
      function_table[l].counter--;
      if (function_table[l].counter==0) {
        function_table[l].counter = function_table[l].period;
        function_table[l].ptr();
      }
    }
  }

  T1IR |= 1;
  VICVectAddr = 0x00000000;
}

uint32_t disable_timer1_irq()
{
  uint32_t old = T1MCR;
  T1MCR &= ~(TMCR_MR0_I);
  return old;
}

uint32_t enable_timer1_irq()
{
  uint32_t old = T1MCR;
  T1MCR |= TMCR_MR0_I;
  return old;
}

uint32_t restore_timer1_irq(uint32_t flag)
{
  uint32_t old = T1MCR;
  T1MCR = (old & ~TMCR_MR0_I) | (flag & TMCR_MR0_I);
  return old;
}

void timer1_add_user_function(TimerFunction tf, uint16_t period)
{
  uint8_t i;

  for (i=0; i<MAX_TIMER1_USER_FUNCTIONS; i++) {
    if (function_table[i].ptr == 0) {
      function_table[i].ptr = tf;
      function_table[i].counter = period;
      function_table[i].period = period;
      break;
    }
  }
}

void timer1_remove_user_function(TimerFunction tf)
{
  uint8_t i;

  for (i=0; i<MAX_TIMER1_USER_FUNCTIONS; i++) {
    if (function_table[i].ptr == tf) {
      function_table[i].ptr = 0;
      break;
    }
  }
}
