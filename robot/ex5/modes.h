#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// Motor move mode
#define IMODE_MOTOR_DEMO    1

/// Sine wave demo
#define IMODE_SINE_DEMO     2

/// The main loop for mode switching
void main_mode_loop(void);

#endif // __MODES_H
