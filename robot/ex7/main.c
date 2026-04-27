/**
 * @file main.c
 * @brief Entry point for experiment 7 firmware (travelling-wave swimming).
 *
 * Boot sequence matches earlier exercises: init hardware and register tables,
 * show life-sign on the head LED, then hand over to the radio-driven mode
 * dispatcher in modes.c.
 */

#include "hardware.h"
#include "registers.h"
#include "modes.h"

int main(void)
{
  hardware_init();
  registers_init();

  /* Green blink at boot — same convention as ex5 */
  set_color_i(2, 0);

  main_mode_loop();

  return 0;
}
