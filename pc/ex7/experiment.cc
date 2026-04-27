/**
 * @file experiment.cc
 * @brief Task 7.2 — PC-side experiment supervisor with LED tracking.
 *
 * Workflow (matches the practical’s suggested protocol):
 *   1. Connect to the radio and to the tracking server.
 *   2. Reboot the head so firmware starts from a known state.
 *   3. Ask the user for swimming parameters (A, nu, phi grid, repetitions).
 *   4. Put the robot in IMODE_SWIM with nu = 0 and A = 0 — only the LED is
 *      meaningful; the animal can be held straight with the tail vertical.
 *   5. Wait for a key press when the experimenter is ready.
 *   6. For each requested trial: stream tracker samples to a CSV, detect the
 *      virtual start line x >= X_START_M, then measure elapsed time until
 *      x >= X_END_M (or timeout).  Cruise speed is chord length / Delta t.
 *   7. After the session, print per-phi mean speed and sample standard deviation.
 *
 * Scientific question: how does total phase lag phi influence average speed?
 *
 * @note Built for the course Windows toolchain (serial + winsock).  Tracking
 *       host name should be verified with the TA (default matches ex6).
 */

#include <windows.h>

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "trkcli.h"
#include "utils.h"

using namespace std;

/* -------------------------------------------------------------------------- */
/*  Protocol constants — adjust if the tank geometry changes                  */
/* -------------------------------------------------------------------------- */

static const char* const TRACKING_PC_NAME = "biorobpc11";
static const uint16_t TRACKING_PORT = 10502;

static const uint8_t RADIO_CHANNEL = 201;
static const char* const INTERFACE = "COM1";

/** REG8_MODE value implemented in robot/ex7 firmware. */
static const uint8_t MODE_SWIM = 3;

/** Radio parameter registers (same encoding as ex5 + byte 22 for phi). */
static const uint16_t REG_FREQ_ENC = 20;
static const uint16_t REG_AMPL_ENC = 21;
static const uint16_t REG_PHI_ENC = 22;

static const float FREQ_MIN = 0.0f;
static const float FREQ_MAX = 1.0f; /* <= 1 Hz constraint */
static const float AMPL_MIN = 0.0f;
static const float AMPL_MAX = 60.0f;
static const float PHI_MIN = 0.5f;
static const float PHI_MAX = 1.25f;

/** Virtual timing gates along the long axis (m). */
static const double X_START_M = 1.0;
static const double X_END_M = 5.5;

/** If the robot never reaches X_END_M, abort after this many seconds. */
static const double TRIAL_TIMEOUT_S = 180.0;

/** Tracker poll period — trkcli already rate-limits to camera fps. */
static const DWORD TRACK_POLL_MS = 15;

/* -------------------------------------------------------------------------- */
static bool get_led_xy(CTrackingClient& trk, double& x, double& y)
{
  uint32_t t = 0;
  if (!trk.update(t)) {
    return false;
  }
  int id = trk.get_first_id();
  if (id == -1 || !trk.get_pos(id, x, y)) {
    return false;
  }
  return true;
}

/** Chord speed (m/s) between two plane samples. */
static double chord_speed_m_s(double x0, double y0, double x1, double y1,
                              double dt_s)
{
  if (dt_s <= 1e-9) {
    return 0.0;
  }
  const double dx = x1 - x0;
  const double dy = y1 - y0;
  return sqrt(dx * dx + dy * dy) / dt_s;
}

static void press_any_key(const char* msg)
{
  cout << msg << endl;
  (void)ext_key();
}

/* -------------------------------------------------------------------------- */
int main()
{
  CRemoteRegs regs;
  CTrackingClient trk;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    return 1;
  }
  if (!trk.connect(TRACKING_PC_NAME, TRACKING_PORT)) {
    regs.close();
    return 1;
  }

  reboot_head(regs);

  float ampl_deg = 0.0f;
  float freq_hz = 0.0f;

  cout << "Half-amplitude A in [ " << AMPL_MIN << " , " << AMPL_MAX << " ] deg: ";
  cin >> ampl_deg;
  if (ampl_deg < AMPL_MIN) {
    ampl_deg = AMPL_MIN;
  }
  if (ampl_deg > AMPL_MAX) {
    ampl_deg = AMPL_MAX;
  }

  cout << "Frequency nu in [ " << FREQ_MIN << " , " << FREQ_MAX << " ] Hz: ";
  cin >> freq_hz;
  if (freq_hz < FREQ_MIN) {
    freq_hz = FREQ_MIN;
  }
  if (freq_hz > FREQ_MAX) {
    freq_hz = FREQ_MAX;
  }

  int n_phi = 0;
  cout << "How many distinct phi values? ";
  cin >> n_phi;
  if (n_phi < 1) {
    n_phi = 1;
  }

  vector<float> phis(static_cast<size_t>(n_phi));
  cout << "Enter each phi in [" << PHI_MIN << "," << PHI_MAX << "] rad:\n";
  for (int i = 0; i < n_phi; i++) {
    cin >> phis[static_cast<size_t>(i)];
  }

  int reps = 3;
  cout << "Repetitions per phi (>=3 recommended): ";
  cin >> reps;
  if (reps < 1) {
    reps = 1;
  }

  string out_prefix;
  cout << "Output file prefix (e.g. run20250427): ";
  cin >> out_prefix;

  /* Idle shape + bright head for tracking. */
  regs.set_reg_b(REG_FREQ_ENC, ENCODE_PARAM_8(0.0f, FREQ_MIN, FREQ_MAX));
  regs.set_reg_b(REG_AMPL_ENC, ENCODE_PARAM_8(0.0f, AMPL_MIN, AMPL_MAX));
  regs.set_reg_b(REG_PHI_ENC, ENCODE_PARAM_8(0.75f, PHI_MIN, PHI_MAX));

  const uint32_t rgb_idle =
      (static_cast<uint32_t>(40) << 16) |
      (static_cast<uint32_t>(64) << 8) |
      static_cast<uint32_t>(40);
  regs.set_reg_dw(REG32_LED, rgb_idle);

  regs.set_reg_b(REG8_MODE, MODE_SWIM);

  press_any_key(
      "Robot is in swim mode at A=0, nu=0. Straighten it, then press any key "
      "to begin the measurement campaign.");

  ofstream summary((out_prefix + "_summary.txt").c_str());
  summary << "# phi_rad N mean_speed_m_s sample_stddev_m_s\n";

  for (size_t pi = 0; pi < phis.size(); pi++) {
    const float phi = phis[pi];

    vector<double> speeds_for_phi;

    for (int r = 0; r < reps; r++) {
      ostringstream csv_name;
      csv_name << out_prefix << "_phi" << fixed << setprecision(3) << phi
               << "_rep" << (r + 1) << ".csv";
      ofstream csv(csv_name.str().c_str());
      csv << "t_unix_s,x_m,y_m,note\n";

      {
        ostringstream os;
        os << "Trial phi=" << phi << " rep " << (r + 1) << "/" << reps
           << " — straighten the lamprey left of x=" << X_START_M
           << " m, then press any key to START swimming.";
        press_any_key(os.str().c_str());
      }

      regs.set_reg_b(REG_PHI_ENC, ENCODE_PARAM_8(phi, PHI_MIN, PHI_MAX));
      regs.set_reg_b(REG_FREQ_ENC, ENCODE_PARAM_8(freq_hz, FREQ_MIN, FREQ_MAX));
      regs.set_reg_b(REG_AMPL_ENC, ENCODE_PARAM_8(ampl_deg, AMPL_MIN, AMPL_MAX));

      bool started = false;
      double t_start = 0.0;
      double x_start = 0.0;
      double y_start = 0.0;
      const double t_arm = time_d();

      cout << "Swimming — first crossing of x>=" << X_START_M
           << " m arms the timer; x>=" << X_END_M << " m stops it.\n";

      while (true) {
        double x = 0.0;
        double y = 0.0;
        const double now = time_d();

        if (!get_led_xy(trk, x, y)) {
          csv << fixed << setprecision(6) << now << ",,,lost\n";
          Sleep(TRACK_POLL_MS);
          if (now - t_arm > TRIAL_TIMEOUT_S) {
            cout << "Timeout (lost spot).\n";
            break;
          }
          continue;
        }

        const char* tag = "approach";
        if (!started) {
          if (x >= X_START_M) {
            started = true;
            t_start = now;
            x_start = x;
            y_start = y;
            tag = "start_line";
            cout << "Start line crossed at t=" << t_start << " s\n";
          }
        } else {
          tag = "trial";
          if (x >= X_END_M) {
            const double v =
                chord_speed_m_s(x_start, y_start, x, y, now - t_start);
            speeds_for_phi.push_back(v);
            cout << "Finish at x=" << x << " m  chord speed ≈ " << v << " m/s\n";
            tag = "finish_line";
            csv << fixed << setprecision(6) << now << "," << x << "," << y << ","
                << tag << "\n";
            break;
          }
        }

        csv << fixed << setprecision(6) << now << "," << x << "," << y << "," << tag
            << "\n";

        if (now - t_arm > TRIAL_TIMEOUT_S) {
          cout << "Timeout before finish line.\n";
          break;
        }

        Sleep(TRACK_POLL_MS);
      }

      /* Coast to idle torque between trials — keeps the same phi loaded. */
      regs.set_reg_b(REG_FREQ_ENC, ENCODE_PARAM_8(0.0f, FREQ_MIN, FREQ_MAX));
      regs.set_reg_b(REG_AMPL_ENC, ENCODE_PARAM_8(0.0f, AMPL_MIN, AMPL_MAX));
      Sleep(500);
    }

    /* Per-phi statistics */
    double mean = 0.0;
    for (size_t i = 0; i < speeds_for_phi.size(); i++) {
      mean += speeds_for_phi[i];
    }
    if (!speeds_for_phi.empty()) {
      mean /= static_cast<double>(speeds_for_phi.size());
    }

    double var = 0.0;
    for (size_t i = 0; i < speeds_for_phi.size(); i++) {
      const double d = speeds_for_phi[i] - mean;
      var += d * d;
    }
    const size_t n = speeds_for_phi.size();
    const double stddev =
        (n > 1) ? sqrt(var / static_cast<double>(n - 1)) : 0.0;

    cout << "\n=== Summary for phi=" << phi << " rad ===\n";
    cout << "N successful runs: " << n << "\n";
    cout << "Mean speed: " << mean << " m/s\n";
    cout << "Sample stdev: " << stddev << " m/s\n\n";

    summary << fixed << setprecision(6) << phi << " " << n << " " << mean << " "
            << stddev << "\n";
  }

  summary.close();

  /* Full stop */
  regs.set_reg_b(REG8_MODE, 0);
  regs.set_reg_b(REG_FREQ_ENC, ENCODE_PARAM_8(0.0f, FREQ_MIN, FREQ_MAX));
  regs.set_reg_b(REG_AMPL_ENC, ENCODE_PARAM_8(0.0f, AMPL_MIN, AMPL_MAX));
  regs.close();

  cout << "Session complete.\n";
  return 0;
}
