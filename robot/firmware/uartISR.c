#include "LPC21xx.h"
#include "uart.h"
#include "uartISR.h"
#include "armVIC.h"
#include "radio.h"

void init_uart0_isr()
{
  VICIntSelect &= ~VIC_BIT(VIC_UART0);    // UART0 selected as IRQ
  VICIntEnable = VIC_BIT(VIC_UART0);      // UART0 interrupt enabled
  VICVectCntl0 = VIC_ENABLE | VIC_UART0;  // UART0 is IRQ0
  VICVectAddr0 = (uint32_t) uart0ISR;
  U0IER = UIER_ERBFI;                     // enable receiver interrupts
}

uint8_t uart0_waitch()
{
  while (!(U0LSR & ULSR_RDR));
  return U0RBR;
}

void uart0ISR(void)
{
  uint8_t iid;

  // perform proper ISR entry so thumb-interwork works properly
  ISR_ENTRY();

  // loop until not more interrupt sources
  while (((iid = U0IIR) & UIIR_NO_INT) == 0)
  {
    // identify & process the highest priority interrupt
    switch (iid & UIIR_ID_MASK)
    {
    case UIIR_RLS_INT:                // Receive Line Status
      U0LSR;                          // read LSR to clear
      break;

    case UIIR_CTI_INT:                // Character Timeout Indicator
    case UIIR_RDA_INT:                // Receive Data Available
      do {
        process_UART_in();
      } while (U0LSR & ULSR_RDR);
      break;

    default:                          // Unknown
      U0LSR;
      U0RBR;
      break;
    }
  }

  VICVectAddr = 0x00000000;             // clear this interrupt from the VIC
  ISR_EXIT();                           // recover registers and return
}
