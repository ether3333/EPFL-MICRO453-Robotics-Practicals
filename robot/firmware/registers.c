#include "registers.h"

void registers_init()
{
  uint8_t i;

  for (i = 0; i < REG8_MAX; i++) {
    reg8_table[i] = 0;
  }
  for (i = 0; i < REG16_MAX; i++) {
    reg16_table[i] = 0;
  }
  for (i = 0; i < REG32_MAX; i++) {
    reg32_table[i] = 0;
  }
}
