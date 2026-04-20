#include <iostream>
#include "robot.h"

using namespace std;

const uint8_t REQ_LOCAL_INTF_VERSION = 5;        ///< required firmware version for local radio interface
const uint8_t REQ_REMOTE_INTF_VERSION = 0x49;    ///< required firmware version for remote radio interface

bool init_radio_interface(const char* port_name, const uint8_t channel, CRemoteRegs& regs)
{
  if (!regs.open(port_name, 57600)) {
    return false;
  }
  if (!regs.sync()) {
    cerr << "Interface synchronization failed!" << endl;
    return false;
  }
  // Tests if the radio interface is there and has the correct firmware
  if (regs.get_reg_b(REG_INTF_VER) != REQ_LOCAL_INTF_VERSION) {
    cerr << "Wrong wireless interface firmware detected." << endl;
    return false;
  }
  
  // Sets the channel number on the radio interface
  regs.set_reg_b(REG_INTF_CH, channel);

  // Verifies if the communication with the remote radio works, and checks
  // that it has the right firmware
  uint8_t ver;
  if (!regs.get_reg_b(REG_RWL_VER, ver)) {
    cerr << "Unable to communicate with the remote module." << endl;
    return false;
  } else if (ver != REQ_REMOTE_INTF_VERSION) {
    cerr << "Wrong remote wireless firmware detected (" << ver << ")." << endl;
    return false;
  }
  
  return true;
}

void reboot_head(CRemoteRegs& regs)
{
  cout << "Rebooting head module... ";
  regs.set_reg_b(REG_BL_CTRL, 1);
  if (regs.get_reg_b(REG_BL_CTRL) != 1) {
    cerr << "Unable to enter bootloader mode on the remote ARM." << endl;
    return;
  }
  regs.set_reg_b(REG_BL_CTRL, 0);
  Sleep(1000);  // makes sure the remote element has the time to initialize...
  cout << "ok." << endl;
}
