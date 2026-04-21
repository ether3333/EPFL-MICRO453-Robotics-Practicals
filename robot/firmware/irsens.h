#ifndef __IRSENS_H
#define __IRSENS_H

#include <stdint.h>

#define IR_LEFT 0
#define IR_FRONT 1
#define IR_RIGHT 2

#define IR_PASSIVE 0
#define IR_ACTIVE 1

void enable_ir_sensor(uint16_t period);
void disable_ir_sensor(void);
void set_ir_mode(uint8_t mode);

extern uint16_t ir_off_val[3];
extern uint16_t ir_on_val[3];

#endif
