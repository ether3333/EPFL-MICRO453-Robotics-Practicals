#ifndef __TRKCLI_H
#define __TRKCLI_H

#include <stdint.h>

struct track_point {
  int id;
  float x;
  float y;
} __attribute__((__packed__));

const int MAX_POINTS = 40;

class CTrackingClient {

public:

  CTrackingClient();
  ~CTrackingClient();

  bool connect(const char* hostname, u_short port);
  
  bool start_tracking_file(const char* filename);
  bool stop_tracking_file(void);
  
  bool update(uint32_t& time);
  bool get_pos(const int id, double& x, double& y);
  int get_first_id();
  const track_point* get_pos_table(int& count) const;

private:

  int sock;
  bool connected;
  
  track_point positions[MAX_POINTS];
  int pos_count;

};

#endif
