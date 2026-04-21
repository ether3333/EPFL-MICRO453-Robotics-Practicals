#include <stdint.h>
#include <math.h>
#include "config.h"
#include "modes.h"
#include "robot.h"
#include "module.h"
#include "registers.h"
#include "hardware.h"
#ifdef HAS_CAN
#include "can.h"
#endif

void init_mode()
{
  uint8_t i;

  robot_init();

#ifdef HARDWARE_V3
  set_color_i(1, 0);

  // Sets current accumulators to zero
  for (i = 0; i < MOD_COUNT; i++) {
    set_reg_value_w(mod_addr[i], MREG16_BATT_ACC, 0);
  }
#endif

#ifdef HARDWARE_V3
  set_color(0);
#endif

  // Return to idle mode
  reg8_table[REG8_MODE] = IMODE_IDLE;
}

void stop_mode()
{
  uint8_t i;

  for (i=0; i<MOD_COUNT; i++) {
    bus_set(mod_addr[i], MREG_SETPOINT, 0);
  }
  pause(HUNDRED_MS * 3);
  for (i=0; i<MOD_COUNT; i++) {
    bus_set(mod_addr[i], MREG_MODE, MODE_IDLE);
  }
  reg8_table[REG8_MODE] = IMODE_IDLE;
}

void bus_test_mode()
{
  uint8_t i;

  while (reg8_table[REG8_MODE] == IMODE_BUS_TEST)
  {
    for (i = 0; i<MOD_COUNT; i++) {
#ifdef HARDWARE_V3
      set_reg_value_dw(mod_addr[i], REG32_LED, 0x002A0A);
#else
      bus_set(mod_addr[i], MREG_HW_OPTIONS, HWO_LED);
#endif
    }
#ifdef HARDWARE_V3
    set_color_i(11, 4);
#else
    set_led(1);
#endif
    pause(HUNDRED_MS);
    for (i = 0; i<MOD_COUNT; i++) {
#ifdef HARDWARE_V3
      set_reg_value_dw(mod_addr[i], REG32_LED, 0);
#else
      bus_set(mod_addr[i], MREG_HW_OPTIONS, 0);
#endif
    }
#ifdef HARDWARE_V3
    set_color(0);
#else
    set_led(0);
#endif
    pause(HUNDRED_MS);

  }

  for (i = 0; i<MOD_COUNT; i++) {
#ifdef HARDWARE_V3
      set_reg_value_dw(mod_addr[i], REG32_LED, 0);
#else
      bus_set(mod_addr[i], MREG_HW_OPTIONS, 0);
#endif
  }
}

void pre_bootloader_mode()
{
#ifdef HARDWARE_V3
  set_rgb(15, 3, 0);
  i2c_set(RGB_ADDR, 8, 0x3F);   // ensures group blinking/PWM is enabled on pins
  i2c_set(RGB_ADDR, 1, 0x25);   // select blinking function instead of group PWM
  i2c_set(RGB_ADDR, 6, 127);    // blinking duty cycle 127/255 (symmetric blink)
  i2c_set(RGB_ADDR, 7, 10);     // set blinking period to 0.45 s  (10+1)/24 s
#else
  set_led(1);
#endif
  while (reg8_table[REG8_MODE] == IMODE_WILL_BOOTLOAD);
#ifdef HARDWARE_V3
  set_rgb(0, 0, 0);
  init_rgb_led();               // removes the blinking...
#else
  set_led(0);
#endif
}

#ifdef HARDWARE_V3
void charger_mode()
{
  uint8_t c;
  int16_t cu;
  int32_t sm;

  if (get_ext_voltage()<=225) return;
  set_color_i(0, 4);       // set medium intensity
  while (get_ext_voltage()>225) {   // abt. 8 V
    sm = 0;
    for (c=0; c<16; c++) {
      cu = get_batt_current();
      sm += cu;
    }
    sm = sm / 16;
    cu = sm;
    if (cu>20) set_color(4);         // red:    > 60 mA (C/10)
    if (cu>6 && cu<18) set_color(8); // orange: 10...60 mA (+ hysteresis)
    if (cu<4) set_color(2);          // green:  < 10 mA
  }
  set_color_i(11, 0);
}

uint8_t test_charger_mode()
{
  return (get_ext_voltage()>225);
}

#endif

#ifdef HARDWARE_V2

void charger_mode()
{
  uint8_t i;

  reg32_table[REG32_LED] = LED_MANUAL;
  for (i = 0; i < MOD_COUNT; i++) {
    bus_set(mod_addr[i], MREG_EXT_DEVICE, 1);
  }
  while (reg8_table[REG8_MODE] == IMODE_BATTERY_CHG) {
    set_led(1);
    pause(HUNDRED_MS * 8);
    set_led(0);
    pause(HUNDRED_MS * 8);
  }
  for (i = 0; i < MOD_COUNT; i++) {
    bus_set(mod_addr[i], MREG_EXT_DEVICE, 0);
  }
}

uint8_t test_charger_mode()
{
  return 0;
}

#endif

void main_mode_loop()
{
  reg8_table[REG8_MODE] = IMODE_IDLE;

  while (1)
  {
    // Verifies if the charging mode has to be enabled
    if (test_charger_mode()) {
      reg8_table[REG8_MODE] = IMODE_BATTERY_CHG;
    }

    switch(reg8_table[REG8_MODE])
    {
      case IMODE_IDLE:
        break;
      case IMODE_INIT:
        init_mode();
        break;
      case IMODE_BATTERY_CHG:
        charger_mode();
        reg8_table[REG8_MODE] = IMODE_IDLE;
        break;
      case IMODE_STOP:
        stop_mode();
        break;
      case IMODE_BUS_TEST:
        bus_test_mode();
        break;
      case IMODE_WILL_BOOTLOAD:
        pre_bootloader_mode();
        break;
      default:
        reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
