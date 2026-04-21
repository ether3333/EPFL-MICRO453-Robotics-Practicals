#include "LPC21xx.h"
#include "hardware.h"
#include "registers.h"
#include "timerISR.h"
#include "can.h"
#include "irsens.h"
#include "adc.h"
#include "armVIC.h"

#if defined(HARDWARE_V3)

static const uint8_t colors_r[] = { 0, 0, 0, 0, 2, 2, 2, 2, 2, 1, 0, 0, 2, 1 };
static const uint8_t colors_g[] = { 0, 0, 2, 2, 0, 0, 2, 2, 1, 2, 1, 2, 0, 0 };
static const uint8_t colors_b[] = { 0, 2, 0, 2, 0, 2, 0, 2, 0, 0, 2, 1, 1, 2 };

static uint8_t led_int = 0;

void init_rgb_led()
{
  i2c_set(RGB_ADDR, 0, 0x00);
  i2c_set(RGB_ADDR, 1, 0x05);
  i2c_set(RGB_ADDR, 6, 0xFF);
  i2c_set(RGB_ADDR, 8, 0x3F);
  set_rgb(0, 0, 0);
}

void set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
  i2c_set(RGB_ADDR, 2, r);
  i2c_set(RGB_ADDR, 3, g);
  i2c_set(RGB_ADDR, 4, b);
}

void set_color(uint8_t c)
{
  reg32_table[REG32_LED] = LED_MANUAL;
  set_rgb(colors_r[c] << led_int, colors_g[c] << led_int, colors_b[c] << led_int);
}

void set_color_i(uint8_t c, uint8_t i)
{
  led_int = i;
  set_color(c);
}

uint16_t get_batt_voltage()
{
  return (((uint16_t) i2c_get(BATT_ADDR, 0x0c) << 8) | i2c_get(BATT_ADDR, 0x0d)) >> 5;
}

int16_t get_batt_current()
{
  int16_t tmp;

  tmp = ((int16_t) i2c_get(BATT_ADDR, 0x0e) << 8) | i2c_get(BATT_ADDR, 0x0f);
  tmp = tmp >> 3;
  return tmp;
}

uint16_t get_ext_voltage()
{
  return adc_read_ch(0);
}

int16_t get_batt_current_acc()
{
  return ((int16_t) i2c_get(BATT_ADDR, 0x10) << 8) | i2c_get(BATT_ADDR, 0x11);
}

int16_t get_temperature()
{
  return (((int16_t) i2c_get(BATT_ADDR, 0x18) << 8) | i2c_get(BATT_ADDR, 0x19)) >> 5;
}

#elif defined(HARDWARE_V2)

void set_led(uint8_t i)
{
  if (i) {
    IO0SET = BIT_EXTLED;
  } else {
    IO0CLR = BIT_EXTLED;
  }
}

#endif // HARDWARE_V2

static void led_timer_isr(void)
{
  static volatile uint8_t ctr = 0;
  static volatile uint8_t st = 0;
  uint32_t regdw_led = reg32_table[REG32_LED];

  uint8_t l = (regdw_led >> 24);

  if (l==0) {           // 0 = ON
#if defined(HARDWARE_V3)
    set_rgb(regdw_led >> 16, regdw_led >> 8, regdw_led & 0xFF);
#elif defined(HARDWARE_V2)
    set_led((regdw_led & 0xffffff) != 0));
#endif
  } else if (l!=255) {  // 255 = override (ignore)
    ctr++;
    if (ctr==l) {
      ctr = 0;
      st = !st;
      if (st) {
#if defined(HARDWARE_V3)
        set_rgb(regdw_led >> 16, regdw_led >> 8, regdw_led & 0xFF);
#elif defined(HARDWARE_V2)
        set_led((regdw_led & 0xffffff) != 0));
#endif
      } else {
#if defined(HARDWARE_V3)
        set_rgb(0, 0, 0);
#elif defined(HARDWARE_V2)
        set_led(0);
#endif
      }
    }
  }
}

void uc_init()
{
  PLLCFG = PLLCFG_MSEL | PLLCFG_PSEL;    // configure the PLL
  PLLCON = PLLCON_PLLE;                  // enable the PLL
  PLLFEED = 0xAA;
  PLLFEED = 0x55;
  IO0CLR = 0xFFFFFFFF;                   // clear all outputs
  IO0DIR = BIT(12);                      // setup I/O pin directions
  while (!(PLLSTAT & PLLSTAT_LOCK));     // wait for PLL locking
  PLLCON = PLLCON_PLLE | PLLCON_PLLC;    // connect the now enabled & locked PLL
  PLLFEED = 0xAA;
  PLLFEED = 0x55;
  PINSEL2 = 0x30;                        // GPIO on TRACEPKT / JTAG
  MAMTIM = MAMTIM_CYCLES;                // configure and enable the MAM
  MAMCR = MAMCR_FULL;
  VPBDIV = VPBDIV_VALUE;                 // set the peripheral bus clock speed
  MEMMAP = MEMMAP_FLASH;                 // map interrupt vectors space in flash
  VICIntEnClear = 0xFFFFFFFF;            // clear all interrupts
  VICIntSelect = 0x00000000;             // clear all FIQ selections
  VICDefVectAddr = (uint32_t) reset;     // point unvectored IRQs to reset()
}

void hardware_init()
{
  uc_init();
  initSysTime();
  i2c_init();

#if defined(HARDWARE_V3)
  init_rgb_led();
#elif defined(HARDWARE_V2)
  IO0DIR = IO0DIR | BIT_EXTLED;   // external LED output
  IO1DIR = IO1DIR | BIT_CHG_EN;   // battery charger enable
  IO0CLR = BIT_EXTLED;
  IO1CLR = BIT_CHG_EN;
#endif

  registers_init();
  radio_init();
  adc_init();

  timer1_init(TIMER1_PERIOD);
  timer1_init_isr();
  timer1_add_user_function(led_timer_isr, 1);

#ifdef HARDWARE_V3
  can_head_init();
#endif

  enableIRQ();

}
