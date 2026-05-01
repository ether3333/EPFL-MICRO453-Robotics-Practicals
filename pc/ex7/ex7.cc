#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <windows.h>

#include "remregs.h"
#include "robot.h"
#include "regdefs.h"
#include "trkcli.h"
#include "utils.h"

using namespace std;

static const char* const TRACKING_PC_NAME = "biorobpc6";  ///< tracking server host
static const uint16_t TRACKING_PORT = 10502;               ///< default tracking TCP port

static const uint8_t RADIO_CHANNEL = 126;   ///< must match robot / interface pairing
static const char* const INTERFACE = "COM3";///< serial port of the wireless base

static const float FREQ_MIN = 0.0f, FREQ_MAX = 1.5f;
static const float AMPL_MIN = 0.0f, AMPL_MAX = 60.0f;
static const float PHASE_MIN = 0.5f, PHASE_MAX = 1.0f;
static const float START = 1.0f, STOP = 5.5f;
static const float TIMEOUT = 60.0f;

bool safe_set_reg_b(CRemoteRegs& regs, const uint8_t reg, const uint8_t val)
{
  for (int i(0); i < 10; i++) {
    if (regs.set_reg_b(reg, val)) {
      return true;
    }
  }
  cerr << "FAILED TO SET REGISTER!" << endl;
  return false;
}

int main()
{
  CRemoteRegs regs;
  CTrackingClient trk;

  if (!init_radio_interface(INTERFACE, RADIO_CHANNEL, regs)) {
    cerr << "Radio init failed.\n";
    return 1;
  }

  if (!trk.connect(TRACKING_PC_NAME, TRACKING_PORT)) {
    cerr << "Tracking connect failed.\n";
    regs.close();
    return 1;
  }

  reboot_head(regs);

  uint32_t rgb =
          (static_cast<uint32_t>(0) << 16) |
          (static_cast<uint32_t>(64) << 8) |
          static_cast<uint32_t>(0);

  regs.set_reg_dw(REG32_LED, rgb);
  regs.set_reg_b(REG8_MODE, 1);

  float freq, ampl, phase;

  cout << "Frequency [0, 1.5] Hz: ";
  cin >> freq;
  if (freq < FREQ_MIN) freq = FREQ_MIN; 
  if (freq > FREQ_MAX) freq = FREQ_MAX; 

  cout << "Amplitude [0, 60.0] deg: ";  
  cin >> ampl;
  if (ampl < AMPL_MIN) ampl = AMPL_MIN; 
  if (ampl > AMPL_MAX) ampl = AMPL_MAX; 

  cout << "Phase [0.5, 1.25] deg: ";
  cin >> phase;
  if (phase < PHASE_MIN) phase = PHASE_MIN; 
  if (phase > PHASE_MAX) phase = PHASE_MAX;

  cout << "Press key to start:";

  ext_key();

  safe_set_reg_b(regs, 20, ENCODE_PARAM_8(freq, FREQ_MIN, FREQ_MAX));
  safe_set_reg_b(regs, 21, ENCODE_PARAM_8(ampl, AMPL_MIN, AMPL_MAX));
  safe_set_reg_b(regs, 22, ENCODE_PARAM_8(phase, PHASE_MIN, PHASE_MAX));

  // Build CSV filename from settings
  std::ostringstream fname;
  fname << "swim"
        << "_f" << std::fixed << std::setprecision(2) << freq
        << "_a" << std::fixed << std::setprecision(1) << ampl
        << "_p" << std::fixed << std::setprecision(2) << phase
        << ".csv";
  std::ofstream csv(fname.str());
  csv << "time_s,x,y\n";

  double x = 0.0;
  double y = 0.0;
  double init_time = time_d();
  double elapsed = 0.0;
  double last_log = -1.0;
  bool stopped = false;

  while(!kbhit() && !stopped){
    uint32_t frame_time = 0;
    if (!trk.update(frame_time)) {
      cerr << "Tracking update failed.\n";
      break;
    }

    int id = trk.get_first_id();
    trk.get_pos(id, x, y);

    if (x < START){
      init_time = time_d();
    }

    elapsed = time_d() - init_time;

    // Log at ~1 Hz
    if (elapsed - last_log >= 0.1) {
      csv << std::fixed << std::setprecision(3) << elapsed
          << "," << x << "," << y << "\n";
      last_log = elapsed;
    }

    if (x > STOP){
      cout << "Final time: " << elapsed << endl;
      stopped = true;
      break;
    }
    if (elapsed > TIMEOUT){
      cout << "Timed out" << endl;
      break;
    }
  }
  for (int i(0); i < 10; i++) {
   regs.set_reg_b(REG8_MODE, 0);
  }  
  ext_key();
  
}

