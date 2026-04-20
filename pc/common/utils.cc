#include "utils.h"

double time_d()
{
  FILETIME ft;
  GetSystemTimeAsFileTime(&ft);
  __int64* val = (__int64*) &ft;
  // FILETIME epoch is Jan. 1, 1601: 134774 days to Jan. 1, 1970
  return static_cast<double>(*val) / 10000000.0 - 11644473600.0;
}

bool kbhit()
{
  HANDLE c_in = GetStdHandle(STD_INPUT_HANDLE);
  DWORD d;
  if (!GetNumberOfConsoleInputEvents(c_in, &d)) {
    return false;
  }

  INPUT_RECORD* v = new INPUT_RECORD[d];
  DWORD count;
  PeekConsoleInput(c_in, v, d, &count);
  
  bool result(false);
  for (unsigned int i(0); i < count; i++) {
    if (v[i].EventType == KEY_EVENT && v[i].Event.KeyEvent.bKeyDown) {
      result = true;
    }
  }

  delete[] v;
  return result;
}

DWORD ext_key()
{
  HANDLE hi = GetStdHandle(STD_INPUT_HANDLE);
  INPUT_RECORD i;
  DWORD w;
  do {
    ReadConsoleInput(hi, &i, 1, &w);
  } while (i.EventType != KEY_EVENT || !i.Event.KeyEvent.bKeyDown);
  return (i.Event.KeyEvent.wVirtualKeyCode << 16) | (i.Event.KeyEvent.uChar.AsciiChar & 0xFF);
}
