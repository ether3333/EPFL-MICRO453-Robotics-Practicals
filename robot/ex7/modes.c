/**
 * @file modes.c
 * @brief Task 7.1 — travelling-wave trajectory generator on the lamprey.
 *
 * Kinematics (practical sheet, tail index i = 0 … N-1 toward head):
 *
 *   theta_i(t) = A * w(i) * sin( 2*pi * ( nu*t + i*phi/N ) )
 *
 * where:
 *   nu  — oscillation frequency (Hz), from radio byte 20
 *   A   — half-amplitude (deg),      from radio byte 21
 *   phi — total phase lag (rad),    from radio byte 22
 *   w(i)— optional taper so the tail moves more than the head (biomimetic)
 *
 * Bytes are encoded with ENCODE_PARAM_8 / DECODE_PARAM_8 from regdefs.h
 * so the PC never sends raw floats.
 *
 * Motor LEDs (body modules) are forced off except the head, which is driven
 * from the PC via REG32_LED for the tracking camera (task 6 / 7).
 *
 * @warning Edit SWIM_MOD_ADDR[] so it matches your robot’s CAN table (see
 *          firmware robot.c mod_addr[] in the full course SDK).
 */

#include "config.h"
#include "modes.h"
#include "robot.h"
#include "module.h"
#include "registers.h"
#include "hardware.h"
#include "can.h"
#include "regdefs.h"

#include <math.h>

#ifndef M_TWOPI
#define M_TWOPI (2.0f * (float)M_PI)
#endif

/* -------------------------------------------------------------------------- */
/*  Robot geometry — MUST match your hardware                                 */
/* -------------------------------------------------------------------------- */

/** Number of body segments in the travelling wave. */
#define SWIM_MOD_COUNT 8

/**
 * CAN addresses from tail (index 0) to head (index SWIM_MOD_COUNT-1).
 * Replace with the addresses from your robot.c if different.
 */
static const uint8_t SWIM_MOD_ADDR[SWIM_MOD_COUNT] = {
    21, 22, 23, 24, 25, 26, 27, 28};

/** Extra wag amplitude at the tail: w(i) = 1 + TAIL_BOOST * i/(N-1). */
static const float TAIL_BOOST = 0.65f;

/* Frequency is capped at 1.0 Hz on decode to respect motor stress limit. */
static const float FREQ_MIN = 0.0f;
static const float FREQ_MAX = 1.0f;

static const float AMPL_MIN = 0.0f;
static const float AMPL_MAX = 60.0f;

/** Allowed phase-lag study interval from the practical (rad). */
static const float PHI_MIN = 0.5f;
static const float PHI_MAX = 1.25f;

/* Radio shadow registers (written from PC, same layout idea as ex5). */
static volatile uint8_t freq_enc = 0;
static volatile uint8_t ampl_enc = 0;
static volatile uint8_t phi_enc = 0;

/* -------------------------------------------------------------------------- */
/**
 * Handles ROP_WRITE_8 from the PC into our parameter shadow registers.
 * Addresses 20, 21, 22 are chosen so ex5-style tools still work for freq/ampl
 * on the first two bytes; the third byte is new for phi.
 */
static int8_t swim_register_handler(uint8_t operation, uint8_t address,
                                    RadioData *radio_data)
{
  if (operation == ROP_WRITE_8) {
    if (address == 20) {
      freq_enc = radio_data->byte;
      return TRUE;
    }
    if (address == 21) {
      ampl_enc = radio_data->byte;
      return TRUE;
    }
    if (address == 22) {
      phi_enc = radio_data->byte;
      return TRUE;
    }
  }
  return FALSE;
}

/* -------------------------------------------------------------------------- */
/**
 * Turns off the RGB LED on every body module except the head element.
 * Uses the CAN accessor from the practical (module.h / can.h).
 */
static void disable_body_leds_except_head(void)
{
  uint8_t j;
  for (j = 0; j + 1 < SWIM_MOD_COUNT; j++) {
    set_reg_value_dw(SWIM_MOD_ADDR[j], MREG32_LED, 0);
  }
}

/* -------------------------------------------------------------------------- */
/**
 * Travelling-wave swimming mode.  Blocks until REG8_MODE != IMODE_SWIM.
 */
static void swimming_mode(void)
{
  uint32_t cycletimer = getSysTICs();
  float my_time = 0.0f;
  uint8_t j;

  /* Initialise every segment: position control ready for setpoints. */
  for (j = 0; j < SWIM_MOD_COUNT; j++) {
    init_body_module(SWIM_MOD_ADDR[j]);
    start_pid(SWIM_MOD_ADDR[j]);
  }

  disable_body_leds_except_head();

#ifdef HARDWARE_V3
  /**
   * Let the PC own the head colour via the radio mirror register.
   * LED_MANUAL stops the timer-driven palette from overwriting reg32_table.
   */
  reg32_table[REG32_LED] = LED_MANUAL;
#endif

  radio_add_reg_callback(swim_register_handler);

  set_color(4); /* red while swimming — easy to see in the tank */

  do {
    float nu = DECODE_PARAM_8(freq_enc, FREQ_MIN, FREQ_MAX);
    float A = DECODE_PARAM_8(ampl_enc, AMPL_MIN, AMPL_MAX);
    float phi = DECODE_PARAM_8(phi_enc, PHI_MIN, PHI_MAX);

    uint32_t dt = getElapsedSysTICs(cycletimer);
    cycletimer = getSysTICs();
    float delta_t = (float)dt / (float)sysTICSperSEC;
    my_time += delta_t;

    const float N = (float)SWIM_MOD_COUNT;

    for (j = 0; j < SWIM_MOD_COUNT; j++) {
      float idx = (float)j;
      float w = 1.0f + TAIL_BOOST * (N > 1.0f ? idx / (N - 1.0f) : 0.0f);
      float phase = M_TWOPI * (nu * my_time + idx * phi / N);
      float angle_deg = A * w * sinf(phase);
      bus_set(SWIM_MOD_ADDR[j], MREG_SETPOINT, DEG_TO_OUTPUT_BODY(angle_deg));
    }
  } while (reg8_table[REG8_MODE] == IMODE_SWIM);

  /* Graceful stop: flatten shape then cut torque. */
  for (j = 0; j < SWIM_MOD_COUNT; j++) {
    bus_set(SWIM_MOD_ADDR[j], MREG_SETPOINT, DEG_TO_OUTPUT_BODY(0.0f));
  }
  pause(HALF_SEC);

  for (j = 0; j < SWIM_MOD_COUNT; j++) {
    bus_set(SWIM_MOD_ADDR[j], MREG_MODE, MODE_IDLE);
  }

  set_color(2);
}

/* -------------------------------------------------------------------------- */
void main_mode_loop(void)
{
  reg8_table[REG8_MODE] = IMODE_IDLE;

  while (1) {
    switch (reg8_table[REG8_MODE]) {
    case IMODE_IDLE:
      break;

    case IMODE_SWIM:
      swimming_mode();
      break;

    default:
      /* Unknown opcode — fall back to a safe state */
      reg8_table[REG8_MODE] = IMODE_IDLE;
      break;
    }
  }
}
