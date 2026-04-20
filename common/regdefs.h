#ifndef __REGDEFS_H
#define __REGDEFS_H

#include <stdint.h>

/// Encodes a floating point number to a byte, within the specified range
#define ENCODE_PARAM_8(p,pmin,pmax) ((uint8_t) ((p - pmin) / (float) (pmax-pmin) * 255.0))
/// Decodes a floating point number from a byte, using the specified range
#define DECODE_PARAM_8(b,pmin,pmax) (pmin + (pmax-pmin) * (float) b / 255.0)

/// Declaration of all 8-bit radio static registers
enum {
  REG8_MODE,
  REG8_MAX
};

/// Declaration of all 16-bit radio static registers
enum {
  REG16_VER,
  REG16_MAX
};

/// Declaration of all 32-bit radio static registers
enum {
  REG32_LED,
  REG32_MAX
};

#endif
