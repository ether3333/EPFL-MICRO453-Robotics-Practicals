#include <stdint.h>
#include "hardware.h"
#include "registers.h"

int main(void)
{
  hardware_init();
  reg32_table[REG32_LED] = LED_MANUAL;  // manual LED control

  while (1) {
      set_rgb(0, 127, 0);
      pause(HALF_SEC);
    
      set_rgb(0, 0, 0);
      pause(HALF_SEC);
  }
  return 0;
}
