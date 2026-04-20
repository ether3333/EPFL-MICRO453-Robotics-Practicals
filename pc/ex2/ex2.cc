#include <iostream>
#include "remregs.h"
#include "robot.h"

using namespace std;

const uint8_t RADIO_CHANNEL = 201;         ///< robot radio channel
const char* INTERFACE = "COM1";            ///< robot radio interface

// Displays the contents of a multibyte register as a list of bytes
void display_multibyte_register(CRemoteRegs& regs, const uint8_t addr)
{
  uint8_t data_buffer[32], len;
  if (regs.get_reg_mb(addr, data_buffer, len)) {
    cout << (int) len << " bytes: ";
    for (unsigned int i(0); i < len; i++) {
      if (i > 0) cout << ", ";
      cout << (int) data_buffer[i];
    }
    cout << endl;
  } else {
    cerr << "Unable to read multibyte register." << endl;
  }
}

void registers_demo(CRemoteRegs& regs)
{
  cout << "get_reg_b(6)  = " << (int) regs.get_reg_b(6) << endl;
  cout << "get_reg_b(21) = " << (int) regs.get_reg_b(21) << endl;
  cout << "get_reg_b(21) = " << (int) regs.get_reg_b(21) << endl;
  cout << "get_reg_b(6)  = " << (int) regs.get_reg_b(6) << endl;
  cout << "get_reg_b(6)  = " << (int) regs.get_reg_b(6) << endl;
  
  regs.set_reg_b(18, 42);
  
  cout << "get_reg_mb(2) = ";
  display_multibyte_register(regs, 2);
  
  uint8_t buffer[10];
  for (size_t i(0); i < sizeof(buffer); i++) {
    buffer[i] = 10 + i;
  }
  regs.set_reg_mb(2, buffer, sizeof(buffer));

  cout << "get_reg_mb(2) = ";
  display_multibyte_register(regs, 2);
  
  regs.set_reg_b(2, 11);
  regs.set_reg_b(3, 22);
  
  cout << "get_reg_mb(2) = ";
  display_multibyte_register(regs, 2);
  
  regs.set_reg_w(7, 2121);
  cout << "get_reg_dw(2) = " << regs.get_reg_dw(2) << endl;
  regs.set_reg_w(7, 1765);
  cout << "get_reg_dw(2) = " << regs.get_reg_dw(2) << endl;
}

int main()
{
  CRemoteRegs regs;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }

  // Reboots the head microcontroller to make sure it is always in the same state
  reboot_head(regs);
  
  // Register display demo
  registers_demo(regs);
  
  regs.close();
  return 0;
}
