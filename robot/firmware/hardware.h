#ifndef __HARDWARE_H
#define __HARDWARE_H

#include "i2c.h"
#include "sysTime.h"
#include "adc.h"

/// Value of reg32_table[REG32_LED]
#define LED_MANUAL 0xFF000000

/// The period at which timer1 should call the interrupt
#define TIMER1_PERIOD 50000

#ifdef HARDWARE_V3
/// I2C address of the RGB LED controller
#define RGB_ADDR 98
/// Initialises the RGB LED controller
void init_rgb_led(void);
/// Sets the current color of RGB the LED
void set_rgb(uint8_t r, uint8_t g, uint8_t b);
/// Sets the color of the RGB LED from a predefined list
void set_color(uint8_t c);
/// Sets the intensity and color of the RGB LED from a predefined list
void set_color_i(uint8_t c, uint8_t i);

/// I2C address of the battery protection circuit (DS2764)
#define BATT_ADDR 52
/// Reads the current battery voltage
uint16_t get_batt_voltage(void);
/// Reads the current battery current
int16_t get_batt_current(void);
/// Reads the external (battery charging) voltage
uint16_t get_ext_voltage(void);
/// Reads the battery current accumulator
int16_t get_batt_current_acc(void);
/// Reads the DS2764 die temperature
int16_t get_temperature(void);

#endif // HARDWARE_V3

#ifdef HARDWARE_V2

/// Pin on P0 for external LED
#define BIT_EXTLED   BIT(15)
/// Pin on P1 for battery charger enable
#define BIT_CHG_EN   BIT(16)

/// Sets the state of the external LED
void set_led(uint8_t i);

#endif // HARDWARE_V2

/// Initialises the hadware of the head (direction registers and RGB LED controller
/// if present), and installs the LED blinking interrupt routine called by timerISR
void hardware_init(void);

/// Initialises the microcontroller (first thing to be done at startup!)
void uc_init(void);

#endif // __HARDWARE_H
