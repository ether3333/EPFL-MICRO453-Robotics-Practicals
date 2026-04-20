#include <iostream>
#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "utils.h"
#include "math.h"

using namespace std;

const uint8_t RADIO_CHANNEL = 201;         ///< robot radio channel
const char* INTERFACE = "COM1";            ///< robot radio interface

const float FREQ_MIN = 0.0f, FREQ_MAX = 1.5f;
const float AMPL_MIN = 0.0f, AMPL_MAX = 60.0f;

int main()
{
  CRemoteRegs regs;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboots the head microcontroller to make sure it is always in the same state
  reboot_head(regs);

  float freq, ampl;

  cout << "Frequency [0, 1.5] Hz: ";
  cin >> freq;
  if (freq < FREQ_MIN) freq = FREQ_MIN; 
  if (freq > FREQ_MAX) freq = FREQ_MAX; 

  cout << "Amplitude [0, 60.0] deg: ";
  cin >> ampl;
  if (ampl < AMPL_MIN) ampl = AMPL_MIN; 
  if (ampl > AMPL_MAX) ampl = AMPL_MAX; 

  regs.set_reg_b(20, ENCODE_PARAM_8(freq, FREQ_MIN, FREQ_MAX));
  
  regs.set_reg_b(21, ENCODE_PARAM_8(ampl, AMPL_MIN, AMPL_MAX));

  regs.set_reg_b(REG8_MODE, 2);

  ext_key();
  regs.set_reg_b(REG8_MODE, 0);

  regs.close();
  return 0;
}
