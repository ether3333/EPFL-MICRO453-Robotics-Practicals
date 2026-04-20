/*
 * remregs.cpp -- class for accessing the robot radio interface over RS-232
 * Alessandro Crespi <alessandro.crespi@epfl.ch>, 20.5.2010
 */

#include <stdio.h>
#include "remregs.h"
#include "wperror.h"

#define ATEND(s)  (&s[strlen(s)])

#ifdef DEBUG
const bool DEBUG = true;
#else
const bool DEBUG = false;
#endif

CRemoteRegs::CRemoteRegs()
{
  InitializeCriticalSection(&mutex);
  hSer = NULL;
}

CRemoteRegs::~CRemoteRegs()
{
  EnterCriticalSection(&mutex);
  close();
  LeaveCriticalSection(&mutex);
  DeleteCriticalSection(&mutex);
}

bool CRemoteRegs::open(const char* portname, int spd)
{
  HANDLE h;
  DCB dcb;
  COMMTIMEOUTS ct;

  h = CreateFile(portname, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING,
    0, 0);
  if (h==INVALID_HANDLE_VALUE) {
    wperror(portname);
    return false;
  }

  GetCommState(h, &dcb);
  dcb.DCBlength = sizeof(dcb);
  dcb.BaudRate = spd;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fRtsControl = RTS_CONTROL_ENABLE;
  dcb.fOutxCtsFlow = FALSE; //TRUE;
  if (!SetCommState(h, &dcb)) {
    wperror("SetCommState");
    CloseHandle(h);
    return false;
  }

  SetupComm(h, 4096, 16);

  ZeroMemory(&ct, sizeof(ct));
  ct.ReadTotalTimeoutMultiplier = 1;
  ct.ReadTotalTimeoutConstant = 5000;
  if (!SetCommTimeouts(h, &ct)) {
    wperror("SetCommTimeouts");
    CloseHandle(h);
    return false;
  }

  hSer = h;
  return true;
}

void CRemoteRegs::close()
{
  if (hSer) CloseHandle(hSer);
  hSer = NULL;
}

bool CRemoteRegs::sync()
{
  uint8_t b(0xFF);
  DWORD l;

  EnterCriticalSection(&mutex);
  for (int i(0); i<24; i++) if (!WriteFile(hSer, &b, 1, &l, NULL)) {
    wperror("WriteFile");
    LeaveCriticalSection(&mutex);
    return false;
  }
  b = 0xAA;
  WriteFile(hSer, &b, 1, &l, NULL);
  do {
    if (!ReadFile(hSer, &b, 1, &l, NULL)) {
      wperror("ReadFile");
      LeaveCriticalSection(&mutex);
      return false;
    }
  } while (b!=0xAA);
  return true;
}

bool CRemoteRegs::reg_op(const uint8_t op, const uint16_t addr, const uint8_t* data,
                         const int len, const bool lock)
{
  uint8_t req[32];

  req[0] = (op << 2) | ((addr & 0x300) >> 8);
  req[1] = (addr & 0xFF);
  for (int i(0); i<len; i++) req[i+2] = data[i];

  if (lock) EnterCriticalSection(&mutex);
  DWORD l;
  if (!WriteFile(hSer, req, len + 2, &l, NULL)) {
    wperror("WriteFile");
    if (lock) LeaveCriticalSection(&mutex);
    return false;
  }
  
  if (!ReadFile(hSer, req, 1, &l, NULL)) {
    wperror("ReadFile");
    if (lock) LeaveCriticalSection(&mutex);
    return false;
  }

  if (lock) LeaveCriticalSection(&mutex);
  return (req[0]==ACK);
}

bool CRemoteRegs::get_reg_b(const uint16_t addr, uint8_t& res)
{
  uint8_t buf;
  DWORD l;
  wl_debug d;
  d.op = 0;
  d.addr = addr;
  d.result = false;
  EnterCriticalSection(&mutex);
  if (!reg_op(0x00, addr, NULL, 0)) {
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  if (!ReadFile(hSer, &buf, 1, &l, NULL)) {
    wperror("ReadFile");
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  res = buf;
  if (DEBUG) {
    d.adata = (const void*) &res;
    d.alen = 1;
    d.result = true;
    debug_dump(&d);
  }
  LeaveCriticalSection(&mutex);
  return true;
}

bool CRemoteRegs::get_reg_w(const uint16_t addr, uint16_t& res)
{
  uint16_t buf;
  DWORD l;
  wl_debug d;
  d.op = 1;
  d.addr = addr;
  d.result = false;
  EnterCriticalSection(&mutex);
  if (!reg_op(0x01, addr, NULL, 0)) {
    LeaveCriticalSection(&mutex);
    if (DEBUG) debug_dump(&d);
    return false;
  }
  if (!ReadFile(hSer, &buf, 2, &l, NULL)) {
    wperror("ReadFile");
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  res = buf;
  if (DEBUG) {
    d.adata = (const void*) &res;
    d.alen = 2;
    d.result = true;
    debug_dump(&d);
  }
  LeaveCriticalSection(&mutex);
  return true;
}

bool CRemoteRegs::get_reg_dw(const uint16_t addr, uint32_t& res)
{
  uint32_t buf;
  DWORD l;
  wl_debug d;
  d.op = 2;
  d.addr = addr;
  d.result = false;
  EnterCriticalSection(&mutex);
  if (!reg_op(0x02, addr, NULL, 0)) {
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  if (!ReadFile(hSer, &buf, 4, &l, NULL)) {
    wperror("ReadFile");
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  res = buf;
  if (DEBUG) {
    d.adata = (const void*) &res;
    d.alen = 4;
    d.result = true;
    debug_dump(&d);
  }
  LeaveCriticalSection(&mutex);
  return true;
}

bool CRemoteRegs::get_reg_mb(const uint16_t addr, uint8_t* data, uint8_t& len)
{
  DWORD l;
  EnterCriticalSection(&mutex);
  wl_debug d;
  d.op = 3;
  d.addr = addr;
  d.result = false;
  if (!reg_op(0x03, addr, NULL, 0)) {
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    return false;
  }
  if (!ReadFile(hSer, &len, 1, &l, NULL)) {
    wperror("ReadFile");
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    len = 0;
    return false;
  }
  if (!ReadFile(hSer, data, len, &l, NULL)) {
    wperror("ReadFile");
    if (DEBUG) debug_dump(&d);
    LeaveCriticalSection(&mutex);
    len = 0;
    return false;
  }
  if (DEBUG) {
    d.adata = (const void*) data;
    d.alen = len;
    d.result = true;
    debug_dump(&d);
  }
  LeaveCriticalSection(&mutex);
  return true;
}

uint8_t CRemoteRegs::get_reg_b(const uint16_t addr)
{
  uint8_t result;
  if (!get_reg_b(addr, result)) return 0xFF;
    else return result;
}

uint16_t CRemoteRegs::get_reg_w(const uint16_t addr)
{
  uint16_t result;
  if (!get_reg_w(addr, result)) return 0xFFFF;
    else return result;
}

uint32_t CRemoteRegs::get_reg_dw(const uint16_t addr)
{
  uint32_t result;
  if (!get_reg_dw(addr, result)) return 0xFFFFFFFF;
    else return result;

}

bool CRemoteRegs::set_reg_b(const uint16_t addr, const uint8_t val)
{
  bool res = reg_op(0x04, addr, &val, 1, true);
  if (DEBUG) {
    wl_debug d;
    d.addr = addr;
    d.op = 4;
    d.rdata = (const void*) &val;
    d.rlen = 1;
    d.result = res;
    debug_dump(&d);
  }
  return res;
}

bool CRemoteRegs::set_reg_w(const uint16_t addr, const uint16_t val)
{
  bool res = reg_op(0x05, addr, (const uint8_t *) &val, 2, true);
  if (DEBUG) {
    wl_debug d;
    d.addr = addr;
    d.op = 5;
    d.rdata = (const void*) &val;
    d.rlen = 2;
    d.result = res;
    debug_dump(&d);
  }
  return res;
}

bool CRemoteRegs::set_reg_dw(const uint16_t addr, const uint32_t val)
{
  bool res = reg_op(0x06, addr, (const uint8_t*) &val, 4, true);
  if (DEBUG) {
    wl_debug d;
    d.addr = addr;
    d.op = 6;
    d.rdata = (const void*) &val;
    d.rlen = 4;
    d.result = res;
    debug_dump(&d);
  }
  return res;
}

bool CRemoteRegs::set_reg_mb(const uint16_t addr, const uint8_t* data, const uint8_t len)
{
  uint8_t buf[32];
  if (len>29) return false;
  buf[0] = len;
  for (int i(0); i<len; i++) buf[i+1] = data[i];
  bool res = reg_op(0x07, addr, buf, len + 1, true);
  if (DEBUG) {
    wl_debug d;
    d.addr = addr;
    d.op = 7;
    d.rdata = (const void*) data;
    d.rlen = len;
    d.result = res;
    debug_dump(&d);
  }
  return res;
}

void CRemoteRegs::debug_dump(const wl_debug* dbg)
{
  char buffer[128];

  *buffer = 0;
  switch (dbg->op) {
    case 0x00:
      sprintf(buffer, "get_reg_b(%u) = ", dbg->addr);
      if (!dbg->result) strcat(buffer, "FAILED"); else sprintf(ATEND(buffer), "%u", *(uint8_t*) dbg->adata);
      break;
    case 0x01:
      sprintf(buffer, "get_reg_w(%u) = ", dbg->addr);
      if (!dbg->result) strcat(buffer, "FAILED"); else sprintf(ATEND(buffer), "%u", *(uint16_t*) dbg->adata);
      break;
    case 0x02:
      sprintf(buffer, "get_reg_dw(%u) = ", dbg->addr);
      if (!dbg->result) strcat(buffer, "FAILED"); else sprintf(ATEND(buffer), "%u", *(uint32_t*) dbg->adata);
      break;
    case 0x03:
      sprintf(buffer, "get_reg_mb(%u) = ", dbg->addr);
      if (!dbg->result) strcat(buffer, "FAILED"); else {
        for (int i(0); i < dbg->alen; i++) sprintf(ATEND(buffer), "%02X ", ((uint8_t*) dbg->adata)[i]);
      }
      break;
    case 0x04:
      sprintf(buffer, "set_reg_b(%u,%u)", dbg->addr, *(uint8_t*) dbg->rdata);
      if (!dbg->result) strcat(buffer, " FAILED");
      break;
    case 0x05:
      sprintf(buffer, "set_reg_w(%u,%u)", dbg->addr, *(uint16_t*) dbg->rdata);
      if (!dbg->result) strcat(buffer, " FAILED");
      break;
    case 0x06:
      sprintf(buffer, "set_reg_dw(%u,%u)", dbg->addr, *(uint32_t*) dbg->rdata);
      if (!dbg->result) strcat(buffer, " FAILED");
      break;
    case 0x07:
      sprintf(buffer, "set_reg_mb(%u,", dbg->addr);
      for (int i(0); i < dbg->rlen; i++) sprintf(ATEND(buffer), "%02X ", ((uint8_t*) dbg->rdata)[i]);
      strcat(buffer, ")");
      if (!dbg->result) strcat(buffer, " FAILED");
      break;
  }

  if (*buffer != 0) puts(buffer);
}
