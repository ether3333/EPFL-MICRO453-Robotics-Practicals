#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "config.h"

/// "Safely" sets a 8-bit register on a module (i.e., writes and reads back, retries if needed)
void safe_bus_set(uint8_t module, uint8_t addr, uint8_t value);

#ifdef HAS_CAN
/// "Safely" sets a 16-bit register on a module (i.e., writes and reads back, retries if needed)
void safe_bus_setw(uint8_t module, uint8_t addr, uint16_t value);
#endif

/// Bus independent 8-bit register read (I2C or CAN depending on hardware)
uint8_t bus_get(uint8_t module, uint8_t addr);
/// Bus independent 8-bit register write (I2C or CAN depending on hardware)
void bus_set(uint8_t module, uint8_t addr, uint8_t value);

/// Resets the zero position and starts the PID controller of an element
void start_pid(uint8_t addr);

#ifdef HAS_LEGS
/// Resets the coordinate system on a leg element
void reset_pos(uint8_t el);

/// Sets the position on a leg, inverting it if needed (based on address LSB)
void set_leg_setpoint(uint8_t addr, int8_t pos);
#endif

/// Initializes the PD controller registers of a body module
void init_body_module(uint8_t addr);

/// Initializes the PD controller registers of a limb module
void init_limb_module(uint8_t addr);

#endif // __ROBOT_H
