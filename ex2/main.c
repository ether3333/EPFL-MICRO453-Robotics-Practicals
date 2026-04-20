#include "hardware.h"
#include "registers.h"

static uint32_t datavar = 0;
static uint8_t mb_data_size = 0;
static uint8_t counter = 0;
static uint8_t shift = 0;
static uint8_t mb_data[MAX_MB_SIZE];

/* Register callback function, handles some new registers on the radio.
 * All these registers are of course completely useless, but it demonstrates how
 * to implement a register callback function, and what it can do.
 */
static int8_t register_handler(uint8_t operation, uint8_t address, RadioData* radio_data)
{
  uint8_t i;
  
  switch (operation)
  {
    case ROP_READ_8:
      if (address == 6) {
        radio_data->byte = counter;
        counter = 0;
        return TRUE;
      } else if (address == 21) {
        counter++;
        radio_data->byte = 21 * counter;
        return TRUE;
      }
      break;
    case ROP_READ_32:
      if (address == 2) {
        radio_data->dword = datavar;
        return TRUE;
      }
      break;
    case ROP_READ_MB:
      if (address == 2) {
        radio_data->multibyte.size = mb_data_size;
        for (i = 0; i < mb_data_size; i++) {
          radio_data->multibyte.data[i] = mb_data[i];
        }
        return TRUE;
      }
      break;
    case ROP_WRITE_8:
      if (address >= 2 && address <= 4) {
        mb_data[address - 2] = radio_data->byte;
        return TRUE;
      }
      if (address == 18) {
        shift = radio_data->byte;
        return TRUE;
      }
      break;
    case ROP_WRITE_16:
      if (address == 7) {
        datavar = (datavar * 3) + radio_data->word;
        return TRUE;
      }
      break;
    case ROP_WRITE_MB:
      if (address == 2) {
        mb_data_size = radio_data->multibyte.size;
        for (i = 0; i < mb_data_size; i++) {
          mb_data[i] = radio_data->multibyte.data[i] + shift;
        }
        return TRUE;
      }
      break;
  }
  return FALSE;
}

int main(void)
{
  hardware_init();
  
  // Registers the register handler callback function
  radio_add_reg_callback(register_handler);
  
  // Changes the color of the led (first red then green) to show the boot
  // and then say that the module is ready.
  set_color_i(4, 0);
  pause(ONE_SEC);
  set_color_i(2, 0);
  
  // Keeps the LED blinking in green to demonstrate that the main program is
  // still running and registers are processed in background.
  while (1) {
    set_color_i(2, 0);
    pause(HALF_SEC);
    set_color_i(0, 0);
    pause(HALF_SEC);
  }

  return 0;
}
