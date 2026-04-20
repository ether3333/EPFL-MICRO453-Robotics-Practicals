#include "hardware.h"
#include "module.h"
#include "robot.h"
#include "registers.h"
#include "radio.h"


// Define how many motors we have
#define MOTOR_COUNT 4
#define POSITION_REG_ADDR 10

volatile int8_t positions[MOTOR_COUNT];

// Addresses of the motors
const uint8_t MOTOR_ADDR[MOTOR_COUNT] = {21, 72, 73, 74};

static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
  uint8_t i;
  if (operation == ROP_READ_MB && address == POSITION_REG_ADDR) {
    radio_data->multibyte.size = MOTOR_COUNT;
    for (i = 0; i < MOTOR_COUNT; i++) {
      radio_data->multibyte.data[i] = positions[i];
    }
    return TRUE;
  }
  return FALSE;
}

int main(void)
{
  uint8_t i;

  hardware_init();
  
  // Changes the color of the led (red) to show the boot
  set_color_i(4, 0);

  // Initialises the body module with the specified address (but do not start
  // the PD controller so the element can be manually moved)
  
  init_body_module(MOTOR_ADDR[0]);
  init_body_module(MOTOR_ADDR[1]);
  init_limb_module(MOTOR_ADDR[2]);
  init_limb_module(MOTOR_ADDR[3]);

  radio_add_reg_callback(register_handler);

  set_color_i(2,0);
  
  // Main loop
  while (1) {

    for (i=0;i<MOTOR_COUNT;i++){
      positions[i] = (int8_t) bus_get(MOTOR_ADDR[i],MREG_POSITION);
    }
  }

  return 0;
}
