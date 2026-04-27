
#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// Sine wave demo
#define IMODE_SWIM          1

/// The main loop for mode switching
void main_mode_loop(void);

#endif // __MODES_H