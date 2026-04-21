#include "config.h"
#include "modes.h"
#include "robot.h"
#include "module.h"
#include "registers.h"
#include "hardware.h"

const float FREQ = 1.0;   // Hz

const uint8_t MOTOR_ADDR = 21;

static volatile uint8_t freq_enc = 0;
static volatile uint8_t ampl_enc = 0;

static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
  if (operation == ROP_WRITE_8) {
    if (address == 20){
      freq_enc = radio_data->byte;
    }
    if (address == 21){
      ampl_enc = radio_data->byte;
    }
    return TRUE;
  }
  return FALSE;
}

void sine_demo_mode()
{
  uint32_t cycletimer = getSysTICs();
  float my_time = 0;

  // Initialize controller and PD parameters
  init_body_module(MOTOR_ADDR);
  // Start the PD controller
  start_pid(MOTOR_ADDR);
  // Set the LED to red to show activity
  set_color(4);

  radio_add_reg_callback(register_handler);

  do {

    float freq = DECODE_PARAM_8(freq_enc, 0.0f, 1.5f);
    float ampl = DECODE_PARAM_8(ampl_enc, 0.0f, 60.0f);

    // Calculates the delta_t in seconds and adds it to the current time
    uint32_t dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    float delta_t = (float) dt / sysTICSperSEC;
    my_time += delta_t;

    // Calculates the sine wave
    float l = ampl * sin(M_TWOPI * freq * my_time);

    bus_set(MOTOR_ADDR, MREG_SETPOINT, DEG_TO_OUTPUT_BODY(l));

  } while (reg8_table[REG8_MODE] == IMODE_SINE_DEMO);

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
  reg8_table[REG8_MODE] = IMODE_IDLE;

  while (1)
  {
    switch(reg8_table[REG8_MODE])
    {
      case IMODE_IDLE:
        break;
      case IMODE_SINE_DEMO:
        sine_demo_mode();
        break;
      default:
        reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
