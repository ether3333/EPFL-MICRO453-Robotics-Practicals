#ifndef __NETUTIL_H
#define __NETUTIL_H

#include <cstdio>
#include <cstring>
#include <stdint.h>
#ifdef _WIN32
  #include <windows.h>
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <netdb.h>
  #include <unistd.h>
#endif

uint32_t gethostaddress(const char* hn);
int block_recv(const int sk, const uint8_t* data, unsigned int len);

#endif

