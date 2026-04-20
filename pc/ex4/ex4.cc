#include <iostream>
#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "utils.h"
#include "math.h"

using namespace std;

const uint8_t RADIO_CHANNEL = 201;         ///< robot radio channel
const char* INTERFACE = "COM1";            ///< robot radio interface

int main()
{
  CRemoteRegs regs;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboots the head microcontroller to make sure it is always in the same state
  reboot_head(regs);


  regs.set_reg_b(REG8_MODE, 2);

  while (!kbhit())

  {
    double t = time_d();
    double angle = 40.0*sin(2.0 * M_PI * 1.0 * t);
    int8_t setpoint = (int8_t)(angle);
    regs.set_reg_b(20, (uint8_t)setpoint);
  }

  ext_key();
  regs.set_reg_b(REG8_MODE, 0);
  cout << endl;

  regs.close();
  return 0;
}
