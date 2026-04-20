#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "remregs.h"

const uint16_t REG_INTF_VER = 0x3C0;       ///< radio interface firmware version
const uint16_t REG_INTF_CH = 0x3C1;        ///< radio interface channel number
const uint16_t REG_RWL_VER = 0x3E0;        ///< remote radio firmware version
const uint16_t REG_BL_CTRL = 0x3E2;        ///< bootloader control register (for reboot)

/// Reboots the head element to make sure everything is reinitialized...
void reboot_head(CRemoteRegs& regs);

/// Initialises the robot radio interface
bool init_radio_interface(const char* port_name, const uint8_t channel, CRemoteRegs& regs);

#endif
