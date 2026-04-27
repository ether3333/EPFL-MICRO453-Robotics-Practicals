#include <math.h>
#include "config.h"
#include "robot.h"
#include "registers.h"
#include "sysTime.h"
#include "module.h"
#include "hardware.h"

#ifdef HAS_CAN
#include "can.h"
#else
#include "i2c.h"
#endif

#define LIST_END 0xFF

// Device control registers (generic)
static const uint8_t gen_regs[][2] = {
  { MREG_MODE,             MODE_IDLE },
  { MREG_CONTROL_TYPE,     CONTROL_POS },
  { MREG_HW_OPTIONS,       HWO_ENC_025 | HWO_INVERT_TORQUE },
  { MREG_SW_OPTIONS,       SWO_SEPARATE_D_TERM | SWO_ANTIRESET_WINDUP },
  { MREG_SETPOINT_SOURCE,  SETPOINT_SRC_I2C },
  { MREG_SETPOINT,         0 },
  { MREG_POSITION,         0 },
  { MREG_SAMPLE_TIME_H,   16 },
  { MREG_SAMPLE_TIME_L,  124 },
  { MREG_ERROR_FLAGS,      0 },
  { MREG_KD_DIV_POS,       4 },
  { MREG_FILTER_POS,       1 },
  { MREG_KP_SPEED,        50 },
  { MREG_KP_DIV_SPEED,     1 },
  { MREG_KI_SPEED,         5 },
  { MREG_KI_DIV_SPEED,    10 },
  { MREG_KP_TORQUE,       40 },
  { MREG_KP_DIV_TORQUE,    5 },
  { MREG_KI_TORQUE,       80 },
  { MREG_KI_DIV_TORQUE,    8 },
  { MREG_DIVIDER_POS,      BODY_POS_DIVIDER },
  { MREG_DIVIDER_TORQUE,   2 },
  { MREG_KD_POS,          30 },
  { MREG_DIVIDER_SPEED,    3 },
  { MREG_KP_POS,         100 },
  { MREG_KP_DIV_POS,       3 },
  { LIST_END,              0 }
};

#ifdef HAS_LEGS
// Device control registers (leg element specific)
static const uint8_t leg_regs[][2] = {
  { MREG_SW_OPTIONS,       SWO_ANTIRESET_WINDUP },
  { MREG_KD_POS,          40 },
  { MREG_DIVIDER_SPEED,    1 },
  { MREG_KP_POS,          80 },
  { MREG_KP_DIV_POS,       2 },
  { MREG_RESET_VALUE,     90 },
  { MREG_DIVIDER_POS,      LEGS_POS_DIVIDER },
  { LIST_END,              0 }
};
#endif

uint8_t bus_get(uint8_t module, uint8_t addr)
{
#ifdef HAS_CAN
  return get_reg_value_b(module, addr);
#else
  return i2c_get(module, addr);
#endif
}

void bus_set(uint8_t module, uint8_t addr, uint8_t value)
{
#ifdef HAS_CAN
  set_reg_value_b(module, addr, value);
#else
  i2c_set(module, addr, value);
#endif
}

void safe_bus_set(uint8_t module, uint8_t addr, uint8_t value)
{
  uint8_t i;

  for (i=0; i<5; i++) {
    bus_set(module, addr, value);
    if (bus_get(module, addr)==value) break;
  }
}

#ifdef HAS_CAN
void safe_bus_setw(uint8_t module, uint8_t addr, uint16_t value)
{
  uint8_t i;

  for (i=0; i<5; i++) {
    set_reg_value_w(module, addr, value);
    if (get_reg_value_w(module, addr)==value) break;
  }
}
#endif

// Send standard configuration registers to a single module
static void send_config(uint8_t addr)
{
  uint8_t idx = 0;

  while (gen_regs[idx][0] != 0xff) {
    safe_bus_set(addr, gen_regs[idx][0], gen_regs[idx][1]);
    idx++;
  }
}

#ifdef HAS_LEGS
static void send_config_leg(uint8_t addr)
{
  uint8_t idx = 0;

  while (leg_regs[idx][0] != 0xff) {
    safe_bus_set(addr, leg_regs[idx][0], leg_regs[idx][1]);
    idx++;
  }
}
#endif

void start_pid(uint8_t addr)
{
  safe_bus_set(addr, MREG_MODE, MODE_IDLE);
  safe_bus_set(addr, MREG_CONTROL_TYPE, CONTROL_POS);
  safe_bus_set(addr, MREG_POSITION, 0);
  safe_bus_set(addr, MREG_SETPOINT, 0);
  safe_bus_set(addr, MREG_MODE, MODE_NORMAL);
}

// Zeroes the torque bias (to be used with motor turned off!)
static void set_torque_bias(uint8_t addr)
{
  uint8_t th, tl;

  safe_bus_set(addr, MREG_MODE, 0);
  th = bus_get(addr, MREG_TORQUE_H);
  tl = bus_get(addr, MREG_TORQUE_L);
  bus_set(addr, MREG_TORQUE_BIAS_H, th);
  bus_set(addr, MREG_TORQUE_BIAS_L, tl);
  bus_set(addr, MREG_ERROR_FLAGS, 0);
}

#ifdef HAS_LEGS
void reset_pos(uint8_t el)
{
  bus_set(el, MREG_MODE, MODE_SETPOINT_RESET);
  while (bus_get(el, MREG_MODE)==MODE_SETPOINT_RESET);
}

void set_leg_setpoint(uint8_t addr, int8_t pos)
{
  if ((addr & 1)) pos = -pos;
  bus_set(addr, MREG_SETPOINT, pos);
}
#endif

void init_body_module(uint8_t addr)
{
  uint8_t j;

  // Clears all the registers to make sure we don't have strange defaults inside
  for (j = 40; j < 100; j++) {
    bus_set(addr, j, 0);
  }
  send_config(addr);
  set_torque_bias(addr);
}

#ifdef HAS_LEGS
void init_limb_module(uint8_t addr)
{
  uint8_t j;

  // Clears all the registers
  for (j = 40; j < 100; j++) {
    bus_set(addr, j, 0);
  }
  send_config(addr);
  send_config_leg(addr);
  set_torque_bias(addr);
}
#endif

