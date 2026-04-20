#include <windows.h>
#include <stdio.h>

void wperror(const char* str)
{
  DWORD err = GetLastError();
  char* msg = NULL;

  FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | 60,
    NULL, err, LANG_NEUTRAL, (char*) &msg, 2048, NULL);
  fprintf(stderr, "%s: %s\n", str, msg);
  LocalFree(msg);
}
