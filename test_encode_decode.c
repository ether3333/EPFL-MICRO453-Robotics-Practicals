#include <stdio.h>
#include <stdint.h>
#include <math.h>

#define ENCODE_PARAM_8(p,pmin,pmax) ((uint8_t) ((p - pmin) / (float) (pmax-pmin) * 255.0))
#define DECODE_PARAM_8(b,pmin,pmax) (pmin + (pmax-pmin) * (float) b / 255.0)

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define M_TWOPI (2.0 * M_PI)

#define FREQ_MIN 0.0f
#define FREQ_MAX 1.5f
#define AMPL_MIN 0.0f
#define AMPL_MAX 60.0f

/* -------------------------------------------------------------------------
 * Part 1: encode/decode roundtrip — verifies the int8_t vs uint8_t bug
 * -------------------------------------------------------------------------*/
static void test_roundtrip(void)
{
    float test_freqs[] = {0.0f, 0.5f, 0.75f, 1.0f, 1.25f, 1.5f};
    float test_ampls[] = {0.0f, 20.0f, 30.0f, 40.0f, 50.0f, 60.0f};
    int n = sizeof(test_freqs) / sizeof(test_freqs[0]);

    printf("=== Part 1: Encode/decode roundtrip ===\n\n");

    printf("Frequency (range [%.1f, %.1f] Hz)\n", FREQ_MIN, FREQ_MAX);
    printf("%-10s  %-8s  %-14s  %-14s\n", "Input", "Encoded", "int8_t decode", "uint8_t decode (fix)");
    printf("--------------------------------------------------------------\n");
    for (int i = 0; i < n; i++) {
        float f = test_freqs[i];
        uint8_t enc = ENCODE_PARAM_8(f, FREQ_MIN, FREQ_MAX);
        int8_t  as_signed   = (int8_t)  enc;
        uint8_t as_unsigned = (uint8_t) enc;
        float decoded_bad = DECODE_PARAM_8(as_signed,   FREQ_MIN, FREQ_MAX);
        float decoded_fix = DECODE_PARAM_8(as_unsigned, FREQ_MIN, FREQ_MAX);
        printf("%-10.2f  %-8u  %-14.4f  %-14.4f\n", f, enc, decoded_bad, decoded_fix);
    }

    printf("\nAmplitude (range [%.1f, %.1f] deg)\n", AMPL_MIN, AMPL_MAX);
    printf("%-10s  %-8s  %-14s  %-14s\n", "Input", "Encoded", "int8_t decode", "uint8_t decode (fix)");
    printf("--------------------------------------------------------------\n");
    for (int i = 0; i < n; i++) {
        float a = test_ampls[i];
        uint8_t enc = ENCODE_PARAM_8(a, AMPL_MIN, AMPL_MAX);
        int8_t  as_signed   = (int8_t)  enc;
        uint8_t as_unsigned = (uint8_t) enc;
        float decoded_bad = DECODE_PARAM_8(as_signed,   AMPL_MIN, AMPL_MAX);
        float decoded_fix = DECODE_PARAM_8(as_unsigned, AMPL_MIN, AMPL_MAX);
        printf("%-10.2f  %-8u  %-14.4f  %-14.4f\n", a, enc, decoded_bad, decoded_fix);
    }
}

/* -------------------------------------------------------------------------
 * Part 2: end-to-end communication simulation
 *
 * Mirrors the actual data path:
 *   PC:    clamp → ENCODE_PARAM_8 → set_reg_b(addr, byte)
 *   Robot: register_handler stores radio_data->byte into uint8_t
 *   Robot: sine_demo_mode decodes and computes setpoint each tick
 * -------------------------------------------------------------------------*/

/* Simulated robot state — matches robot/ex5/modes.c after the fix */
static uint8_t robot_freq_enc = 0;
static uint8_t robot_ampl_enc = 0;

/* Simulates set_reg_b: encodes on the PC side and "transmits" a byte */
static void pc_send_params(float freq, float ampl)
{
    /* Clamp — mirrors the bounds checks in ex5.cc */
    if (freq < FREQ_MIN) freq = FREQ_MIN;
    if (freq > FREQ_MAX) freq = FREQ_MAX;
    if (ampl < AMPL_MIN) ampl = AMPL_MIN;
    if (ampl > AMPL_MAX) ampl = AMPL_MAX;

    uint8_t freq_byte = ENCODE_PARAM_8(freq, FREQ_MIN, FREQ_MAX);
    uint8_t ampl_byte = ENCODE_PARAM_8(ampl, AMPL_MIN, AMPL_MAX);

    printf("  PC sends:  freq_byte=%-3u  ampl_byte=%-3u\n", freq_byte, ampl_byte);

    /* Simulates the radio transmission: byte arrives at the robot unchanged */
    robot_freq_enc = freq_byte;   /* register_handler: freq_enc = radio_data->byte */
    robot_ampl_enc = ampl_byte;   /* register_handler: ampl_enc = radio_data->byte */
}

/* Simulates one call to DECODE inside sine_demo_mode */
static void robot_decode_params(float *freq_out, float *ampl_out)
{
    *freq_out = DECODE_PARAM_8(robot_freq_enc, FREQ_MIN, FREQ_MAX);
    *ampl_out = DECODE_PARAM_8(robot_ampl_enc, AMPL_MIN, AMPL_MAX);
    printf("  Robot sees: freq=%.4f Hz  ampl=%.4f deg\n", *freq_out, *ampl_out);
}

/* Simulates a few ticks of the sine loop and prints the motor setpoints */
static void robot_run_sine(float freq, float ampl, int n_ticks, float dt)
{
    printf("  Motor setpoints (%.0f ticks, dt=%.3f s):\n", (float)n_ticks, dt);
    float t = 0.0f;
    for (int i = 0; i < n_ticks; i++) {
        float setpoint = ampl * sin(M_TWOPI * freq * t);
        printf("    t=%.3f s  setpoint=%.2f deg\n", t, setpoint);
        t += dt;
    }
}

static void test_end_to_end(void)
{
    printf("\n=== Part 2: End-to-end communication simulation ===\n");

    /* Pairs of (freq Hz, ampl deg) that cover the problematic range (encoded > 127) */
    float cases[][2] = {
        {0.5f,  20.0f},   /* both encoded < 128: worked before and after fix */
        {1.0f,  40.0f},   /* both encoded > 128: broken before fix           */
        {1.5f,  60.0f},   /* maximum values                                  */
        {-0.5f, 70.0f},   /* out-of-range inputs: should be clamped          */
    };
    int n = sizeof(cases) / sizeof(cases[0]);

    for (int i = 0; i < n; i++) {
        float req_freq = cases[i][0];
        float req_ampl = cases[i][1];
        printf("\n--- Test %d: requested freq=%.2f Hz, ampl=%.2f deg ---\n",
               i + 1, req_freq, req_ampl);

        pc_send_params(req_freq, req_ampl);

        float recv_freq, recv_ampl;
        robot_decode_params(&recv_freq, &recv_ampl);

        float freq_err = fabsf(recv_freq - fmaxf(fminf(req_freq, FREQ_MAX), FREQ_MIN));
        float ampl_err = fabsf(recv_ampl - fmaxf(fminf(req_ampl, AMPL_MAX), AMPL_MIN));
        printf("  Error:    freq_err=%.4f Hz  ampl_err=%.4f deg  %s\n",
               freq_err, ampl_err,
               (freq_err < 0.01f && ampl_err < 0.5f) ? "[PASS]" : "[FAIL]");

        /* Show a few sine setpoints for the interesting cases */
        if (req_freq > 0.0f && req_ampl > 0.0f)
            robot_run_sine(recv_freq, recv_ampl, 5, 0.1f);
    }
}

int main(void)
{
    test_roundtrip();
    test_end_to_end();
    return 0;
}
