#include "hardware.h"
#include "registers.h"
#include "modes.h"

int main(void)
{
  hardware_init();
  registers_init();

  // Changes the color of the led (green) to show the boot
  set_color_i(2, 0);

  // Calls the main mode loop (see modes.c)
  main_mode_loop();

  return 0;
}
