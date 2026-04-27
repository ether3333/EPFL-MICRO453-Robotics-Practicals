#ifndef __MODES_H
#define __MODES_H

/// Idle mode: do nothing
#define IMODE_IDLE          0

/// Motor move mode (reserved / legacy)
#define IMODE_MOTOR_DEMO    1

/// Single-joint sine demo (same numbering as ex5)
#define IMODE_SINE_DEMO     2

/**
 * Task 7 — travelling-wave swimming.
 * The PC sets REG8_MODE to this value after uploading parameters on registers
 * 20 (frequency), 21 (half-amplitude A), 22 (total phase lag phi).
 */
#define IMODE_SWIM          3

void main_mode_loop(void);

#endif /* __MODES_H */
