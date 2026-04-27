#ifndef __REGISTERS_H
#define __REGISTERS_H

#include "radio.h"
#include "regdefs.h"

extern volatile uint8_t reg8_table[];      ///< 8-bit register bank
extern volatile uint16_t reg16_table[];    ///< 16-bit register bank
extern volatile uint32_t reg32_table[];    ///< 32-bit register bank

/// Sets all registers to zero
void registers_init(void);

#endif // __REGISTERS_H
