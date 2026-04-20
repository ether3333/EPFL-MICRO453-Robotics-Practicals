#include <iostream>
#include "netutil.h"

using namespace std;

uint32_t gethostaddress(const char* hn)
{
  struct hostent* hen;
  uint32_t addr;

  // tries to parse the host name as a numerical IP address
  addr = inet_addr(hn);
  if (addr != INADDR_ANY && addr != INADDR_NONE) {
    return addr;
  }

  // if not successful, look up the host name in the DNS
  hen = gethostbyname(hn);
  if (hen != NULL) {
    // sanity check... avoid unexplained crashes...
    if (hen->h_length != sizeof(addr)) {
      cerr << "Internal error: invalid address size." << endl;
      return INADDR_NONE;
    }
    memcpy(&addr, hen->h_addr_list[0], sizeof(addr));
    return addr;
  } else {
    // nothing found
    return INADDR_NONE;
  }
}

// Receives a block of data of the specified size
int block_recv(const int sk, const uint8_t* data, unsigned int len)
{
  int i, j;

  j = 0;

  while (len > 0) {
    i = recv(sk, (char*) data, len, 0);
    if (i==0 || i==-1) return -2;
    data += i;
    len -= i;
    j += i;
  }

  return j;
}
