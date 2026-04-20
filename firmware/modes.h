#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// Initialisation mode: initialises all the elements and goes back to idle
#define IMODE_INIT          1

/// Initialization mode implementation
void init_mode(void);

/// Stop mode: stops the motors and goes back to idle mode
#define IMODE_STOP          3

/// Stop mode implementation
void stop_mode(void);

/// Battery charging mode (possibly automatically enabled): charges the robot
/// or just monitors the charging process (depending on hardware)
#define IMODE_BATTERY_CHG   4

/// Tests for automatically entering charging mode (if supported by hardware)
uint8_t test_charger_mode(void);

/// Battery charging mode
void charger_mode(void);

/// Bus test mode: blinks the LEDs of all the elements
#define IMODE_BUS_TEST      6

/// Bus test mode implementation (i.e., blinks all LEDs on elements)
void bus_test_mode(void);

/// Pre-bootload mode: entered at least 100 ms before flashing the ARM
#define IMODE_WILL_BOOTLOAD 254

/** \brief This mode is activated by the bootloader just before flashing the ARM
 *  \warning The code should run whatever it needs in less than 100 ms.
 */
void pre_bootloader_mode(void);

/// The main loop for mode switching
void main_mode_loop(void);

#endif // __MODES_H
