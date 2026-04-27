#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

uint16_t unaligned_read_16(uint8_t* ptr);
uint32_t unaligned_read_32(uint8_t* ptr);
void unaligned_write_16(uint8_t* ptr, uint16_t val);
void unaligned_write_32(uint8_t* ptr, uint32_t val);

#define NULL 0

#endif // __UTILS_H
