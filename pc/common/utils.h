#ifndef __UTILS_H
#define __UTILS_H

#include <windows.h>
#include <stdint.h>

/// Returns the current UNIX time (as time() does) including the fractional part
double time_d();

/// \brief Checks if a key press is waiting in the keyboard input buffer
/// \return true if a key has been pressed, false otherwise
/// \note Remember that the key is not removed from the input buffer! Use ext_key()
///   to retrieve or discard it, or call FlushConsoleInputBuffer().
bool kbhit();

/// \brief Returns the key code of the first key pressed (including non-character
///   keys as F1, etc.)
/// \return The virtual key code in the upper 16 bits (for a list of the key codes,
///   see http://msdn.microsoft.com/en-us/library/windows/desktop/dd375731%28v=vs.85%29.aspx);
///   the ASCII character (if any) in the lower 16 bits.
DWORD ext_key();

#endif
