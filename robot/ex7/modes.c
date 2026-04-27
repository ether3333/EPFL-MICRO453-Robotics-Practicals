#include "config.h"
#include "modes.h"
#include "robot.h"
#include "module.h"
#include "registers.h"
#include "hardware.h"
#include "can.h"

const float FREQ = 1.0;   // Hz

#define MOTOR_COUNT 5
const uint8_t MOTOR_ADDR[MOTOR_COUNT] = {5, 26, 24, 22, 25};


static volatile uint8_t freq_enc = 0;
static volatile uint8_t ampl_enc = 0;
static volatile uint8_t phase_enc = 0;

float freq = 0;
float ampl = 0;
float phase = 0;

static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
  if (operation == ROP_WRITE_8) {
    if (address == 20){
      freq_enc = radio_data->byte;
      freq = DECODE_PARAM_8(freq_enc, 0.0f, 1.5f);
      return TRUE;
    }
    if (address == 21){
      ampl_enc = radio_data->byte;
      ampl = DECODE_PARAM_8(ampl_enc, 0.0f, 60.0f);
      return TRUE;
    }
    if (address == 22){
        phase_enc = radio_data->byte;
        phase = DECODE_PARAM_8(phase_enc, 0.5f, 1.0f);
        return TRUE;
    }
    return FALSE;
  }
  return FALSE;
}

void swim_mode()
{
  uint32_t cycletimer = getSysTICs();
  float my_time = 0;

  for (int i = 0; i < MOTOR_COUNT; i++){
    init_body_module(MOTOR_ADDR[i]);
  }
  for (int i = 0; i < MOTOR_COUNT; i++){
    start_pid(MOTOR_ADDR[i]);
  }
  for (int i = 0; i < MOTOR_COUNT; i++){
    set_reg_value_dw(MOTOR_ADDR[i], MREG32_LED, 0);
  }

  do {

    // Calculates the delta_t in seconds and adds it to the current time
    uint32_t dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    float delta_t = (float) dt / sysTICSperSEC;
    my_time += delta_t;

    for (int i = 0; i < MOTOR_COUNT; i++){
        float theta = ampl * sin(M_TWOPI*(freq*my_time + i*phase/MOTOR_COUNT));
        bus_set(MOTOR_ADDR[i], MREG_SETPOINT, DEG_TO_OUTPUT_BODY(theta));
    }
    
  } while (reg8_table[REG8_MODE] == IMODE_SWIM);

  // Send back the motor to zero position
  for (int i = 0; i < MOTOR_COUNT; i++){
    bus_set(MOTOR_ADDR[i], MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0));
  }
  // Give it some time for this to happen
  pause(HALF_SEC);
  // Turn off the PD controller
  for (int i = 0; i < MOTOR_COUNT; i++){
    bus_set(MOTOR_ADDR[i], MREG_MODE, MODE_IDLE);
  }
  
}

void main_mode_loop()
{
  reg8_table[REG8_MODE] = IMODE_IDLE;
  radio_add_reg_callback(register_handler);
  while (1)
  {
    switch(reg8_table[REG8_MODE])
    {
      case IMODE_IDLE:
        break;
      case IMODE_SWIM:
        swim_mode();
        break;
      default:
        reg8_table[REG8_MODE] = IMODE_IDLE;
    }
  }
}
