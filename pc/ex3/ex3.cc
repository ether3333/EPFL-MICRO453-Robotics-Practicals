#include <iostream>
#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "utils.h"

using namespace std;

const uint8_t RADIO_CHANNEL = 201;         ///< robot radio channel
const char* INTERFACE = "COM1";            ///< robot radio interface

const uint8_t POSITIONS_REG_ADDR = 10;

int main()
{
  CRemoteRegs regs;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboots the head microcontroller to make sure it is always in the same state
  reboot_head(regs);

  while (!kbhit())
  {
    uint8_t data[32];
    uint8_t len;

    if (regs.get_reg_mb(POSITIONS_REG_ADDR,data,len))
    {
      for (uint8_t i = 0; i < len; i++)
      {
        cout << "Motor " << (int)i << ": " << (int)((int8_t)data[i]) << "\t";

      }
      cout << endl;
      
    }
  }
  ext_key();
  cout << endl;

  regs.close();
  return 0;
}
