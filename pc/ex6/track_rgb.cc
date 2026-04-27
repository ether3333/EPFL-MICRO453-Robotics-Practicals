/**
 * @file track_rgb.cc
 * @brief Task 6.1 — Combine LED spot tracking with the radio head LED (RGB).
 *
 * The aquarium tracking PC streams (x, y) of the head LED in metres.  This
 * program maps position to colour while keeping green fixed at 64 so the
 * blob stays easy for the tracker to segment:
 *
 *   - red   increases with x across the pool width  (~0 … 6 m)
 *   - blue  increases with y across the pool depth (~0 … 2 m)
 *
 * The 32-bit word written to the head is (r << 16) | (g << 8) | b as in the
 * practical sheet.  Register index REG32_LED is 0 in regdefs.h.
 *
 * @note Run on the lab Windows PC with the radio dongle; set INTERFACE and
 *       RADIO_CHANNEL to match your bench.  Tracking hostname/port match ex6.
 */

#include <iostream>
#include <cstdint>
#include <windows.h>

#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "trkcli.h"
#include "utils.h"

using namespace std;

/* -------------------------------------------------------------------------- */
/*  Bench configuration — edit for your session                               */
/* -------------------------------------------------------------------------- */

static const char* const TRACKING_PC_NAME = "biorobpc6";  ///< tracking server host
static const uint16_t TRACKING_PORT = 10502;               ///< default tracking TCP port

static const uint8_t RADIO_CHANNEL = 126;   ///< must match robot / interface pairing
static const char* const INTERFACE = "COM3";///< serial port of the wireless base

/** Physical size of the visible tank (m), used to map position → colour. */
static const double AQUARIUM_X_M = 6.0;
static const double AQUARIUM_Y_M = 2.0;

/** Fixed green component for reliable chroma segmentation (assignment). */
static const uint8_t G_FIXED = 64;

/* -------------------------------------------------------------------------- */
/** Clamp @a v into [0, 255] after linear scaling from [0, span]. */
static uint8_t scale_axis(double pos_m, double span_m)
{
  if (span_m <= 0.0) {
    return 0;
  }
  double t = pos_m / span_m;
  if (t < 0.0) {
    t = 0.0;
  }
  if (t > 1.0) {
    t = 1.0;
  }
  return static_cast<uint8_t>(t * 255.0 + 0.5);
}

/* -------------------------------------------------------------------------- */
int main()
{
  CRemoteRegs regs;
  CTrackingClient trk;

  /* Radio first — head must be running firmware that exposes REG32_LED. */
  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    cerr << "Radio init failed.\n";
    return 1;
  }

  if (!trk.connect(TRACKING_PC_NAME, TRACKING_PORT)) {
    cerr << "Tracking connect failed.\n";
    regs.close();
    return 1;
  }

  cout << "Tracking + radio active.  Move the LED; press any key to quit.\n";

  while (!kbhit()) {
    uint32_t frame_time = 0;

    if (!trk.update(frame_time)) {
      cerr << "Tracking update failed.\n";
      break;
    }

    int id = trk.get_first_id();
    double x = 0.0;
    double y = 0.0;

    if (id != -1 && trk.get_pos(id, x, y)) {
      /* Map aquarium coordinates to R and B; G stays constant. */
      uint8_t r = scale_axis(x, AQUARIUM_X_M);
      uint8_t b = scale_axis(y, AQUARIUM_Y_M);
      uint32_t rgb =
          (static_cast<uint32_t>(r) << 16) |
          (static_cast<uint32_t>(G_FIXED) << 8) |
          static_cast<uint32_t>(b);

      regs.set_reg_dw(REG32_LED, rgb);

      cout << fixed;
      cout.precision(2);
      cout << "x=" << x << " m  y=" << y << " m   RGB("
           << static_cast<int>(r) << ","
           << static_cast<int>(G_FIXED) << ","
           << static_cast<int>(b) << ")\r";
    } else {
      cout << "(no spot)                    \r";
    }

    Sleep(10);  /* ~100 Hz poll; tracker is ~15 Hz so this is plenty */
  }

  FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE));
  regs.close();
  return 0;
}
