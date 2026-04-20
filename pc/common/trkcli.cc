/*
 * trkcli.cpp -- class for accessing the LED tracking system over TCP/IP
 * Alessandro Crespi <alessandro.crespi@epfl.ch>, 15.4.2009
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#ifdef _WIN32
  #include <winsock.h>
  #include "wperror.h"
  #define perror wperror
#else
  #include <unistd.h>
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <netdb.h>
  #include <sys/socket.h>
  #include <sys/types.h>
  #define closesocket ::close
#endif

#include "trkcli.h"
#include "netutil.h"

CTrackingClient::CTrackingClient()
{
#ifdef _WIN32
  WSADATA ws;
  WSAStartup(0x0101, &ws);
#endif
  connected = false;
  pos_count = 0;
  memset(positions, 0, sizeof(positions));
}

CTrackingClient::~CTrackingClient()
{
  if (connected) closesocket(sock);
#ifdef _WIN32
  WSACleanup();
#endif
}

bool CTrackingClient::connect(const char* hostname, u_short port)
{
  if (connected) {
    fprintf(stderr, "Tracking client already connected.\n");
    return false;
  }
  u_long IP = gethostaddress(hostname);
  if (IP==INADDR_NONE) {
    fprintf(stderr, "Invalid hostname: %s.\n", hostname);
    return false;
  }
  sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (sock==-1) return false;

  struct sockaddr_in sai;

  sai.sin_family = AF_INET;
  sai.sin_port = htons(port);
  sai.sin_addr.s_addr = IP;

  if (::connect(sock, (sockaddr*) &sai, sizeof(sai))==-1) {
    fprintf(stderr, "Unable to connect to %s:%d.\n", hostname, port);
    closesocket(sock);
    return false;  
  }

  int i;
  i = 1;
#ifdef _WIN32
  setsockopt(sock, SOL_SOCKET, TCP_NODELAY, (char*) &i, sizeof(i));
#else
  setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, (char*) &i, sizeof(i));
#endif
  connected = true;

  return true;
}

bool CTrackingClient::start_tracking_file(const char* filename)
{
  if (!connected) return false;

  int len = strlen(filename) + 2;
  char* buf = new char[len + 1];
  
  buf[0] = 'S';
  buf[1] = strlen(filename);
  strcpy(&buf[2], filename);
  if (send(sock, buf, len, 0)!=len) {
    perror("send");
    closesocket(sock);
    connected = false;
    return false;
  }
  
  if (recv(sock, buf, 1, 0)!=1) {
    perror("recv");
    closesocket(sock);
    connected = false;
    return false;
  }
  
  if (buf[0]!='+') {
    fprintf(stderr, "Tracking start failed on the server.\n");
    return false;
  } else return true;

}

bool CTrackingClient::stop_tracking_file()
{
  if (!connected) return false;

  char c('s');
  if (send(sock, &c, 1, 0)!=1) {
    perror("send");
    closesocket(sock);
    connected = false;
    return false;
  }
  if (recv(sock, &c, 1, 0)!=1) {
    perror("recv");
    closesocket(sock);
    connected = false;
    return false;
  }
  if (c!='+') {
    fprintf(stderr, "Tracking stop failed on the server.\n");
    return false;
  } else return true;
  
}
  
bool CTrackingClient::update(uint32_t& time)
{
  if (!connected) return false;
  
  char c('U');
  if (send(sock, &c, 1, 0)!=1) {
    perror("send");
    closesocket(sock);
    connected = false;
    return false;
  }
  if (recv(sock, (char*) &time, 4, 0)!=4) {
    perror("recv");
    closesocket(sock);
    connected = false;
    return false;
  }
  if (recv(sock, &c, 1, 0)!=1) {
    perror("recv");
    closesocket(sock);
    connected = false;
    return false;
  }
  if (c > MAX_POINTS) {
    fprintf(stderr, "Error: server wants to return %d spots, only %d allowed.\n", c, MAX_POINTS);
    fprintf(stderr, "Connection closed.");
    closesocket(sock);
    connected = false;
    return false;
  }
  int size = (int) c * sizeof(track_point);
  if (block_recv(sock, (uint8_t*) positions, size)!=size) {
    fprintf(stderr, "Error while receiving data block from server, closing connection.\n");
    closesocket(sock);
    connected = false;
    return false;
  } else pos_count = c;
  
  return true;
}


bool CTrackingClient::get_pos(const int id, double &x, double &y)
{
  for (int i(0); i<pos_count; i++) {
    if (positions[i].id==id) {
      x = positions[i].x;
      y = positions[i].y;
      return true;
    }
  }
  return false;
}

int CTrackingClient::get_first_id()
{
  if (pos_count>0)
    return positions[0].id;
  else
    return -1;
}

const track_point* CTrackingClient::get_pos_table(int& count) const
{
  count = pos_count;
  return positions;
}
