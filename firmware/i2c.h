#ifndef __I2C_H
#define __I2C_H

#include <stdint.h>

void i2c_init(void);
void i2c_reset(void);
uint8_t i2c_get(uint8_t addr, uint8_t reg);
uint8_t i2c_set(uint8_t addr, uint8_t reg, uint8_t val);

#endif
