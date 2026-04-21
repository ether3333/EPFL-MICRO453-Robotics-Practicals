#include "config.h"
#include "modes.h"
#include "robot.h"
#include "module.h"
#include "registers.h"
#include "hardware.h"

// Address of the motor we control
const uint8_t MOTOR_ADDR = 21;

void motor_demo_mode()
{
  // Initialize controller and PD parameters
  init_body_module(MOTOR_ADDR);
  // Start the PD controller
  start_pid(MOTOR_ADDR);
  // Set the LED to red to show activity
  set_color(4);
  // Loop until we remain in demo mode
  while (reg8_table[REG8_MODE] == IMODE_MOTOR_DEMO) {
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(21.0));
    pause(HALF_SEC);
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(-21.0));
    pause(HALF_SEC);
  }
  // Send back the motor to zero position
  bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0));
  // Give it some time for this to happen
  pause(HALF_SEC);
  // Turn off the PD controller
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);
  // Set back LED to green
  set_color(2);
}

static volatile int8_t setpoint = 0;

static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
  if (operation == ROP_WRITE_8 && address == 20) {
    setpoint = radio_data->byte;
    return TRUE;
  }
  return FALSE;
}

void pc_setpoint_mode()
{
  // Initialize controller and PD parameters
  init_body_module(MOTOR_ADDR);
  // Start the PD controller
  start_pid(MOTOR_ADDR);
  // Set the LED to red to show activity
  set_color(4);
  
  radio_add_reg_callback(register_handler);
  
  // Loop until we remain in demo mode
  while (reg8_table[REG8_MODE] == PC_SETPOINT_MODE) {
    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(setpoint));
  }
  // Send back the motor to zero position
  bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0));
  // Give it some time for this to happen
  pause(HALF_SEC);
  // Turn off the PD controller
  bus_set(MOTOR_ADDR, MREG_MODE, MODE_IDLE);
  // Set back LED to green
  set_color(2);
}


void main_mode_loop()
{
  // Idle mode must be the default at startup
  reg8_table[REG8_MODE] = IMODE_IDLE;

  // Main loop
  while (1)
  {
    switch(reg8_table[REG8_MODE])
    {
      case IMODE_IDLE:
        break;
      case IMODE_MOTOR_DEMO:
        motor_demo_mode();
        break;
      case PC_SETPOINT_MODE:
        pc_setpoint_mode();
        break;
      default:
        reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}


