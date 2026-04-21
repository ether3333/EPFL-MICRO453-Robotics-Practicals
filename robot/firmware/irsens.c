#include "irsens.h"
#include "timerisr.h"
#include "hardware.h"

uint16_t ir_off_val[3];
uint16_t ir_on_val[3];

static uint8_t ir_mode = IR_PASSIVE;

static void ir_sensor_callback(void);

void enable_ir_sensor(uint16_t period)
{
  timer1_add_user_function(ir_sensor_callback, period);
  IO0DIR = IO0DIR | BIT(15);
  IO0CLR = BIT(15);
}

void disable_ir_sensor(void)
{
  timer1_remove_user_function(ir_sensor_callback);
}

void set_ir_mode(uint8_t mode)
{
  ir_mode = mode;
}

void ir_sensor_callback()
{
  uint8_t i;

  for (i=0; i<3; i++) {
    adc_set_channel(i+1);
    pause(100);
    ir_off_val[i] = adc_read();
  }

  if (ir_mode == IR_ACTIVE) {
    IO0SET = BIT(15);
    pause(2000);  // 200 microseconds
    for (i=0; i<3; i++) {
      adc_set_channel(i+1);
      pause(100);
      ir_off_val[i] = adc_read();
    }
    IO0CLR = BIT(15);
  }
}
