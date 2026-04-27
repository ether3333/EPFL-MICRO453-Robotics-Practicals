#include "utils.h"

uint16_t unaligned_read_16(uint8_t* ptr)
{
  uint16_t result;

  uint8_t* i = (uint8_t*) ptr;
  uint8_t* j = (uint8_t*) &result;

  j[0] = i[0];
  j[1] = i[1];

  return result;
}

uint32_t unaligned_read_32(uint8_t* ptr)
{
  uint32_t result;

  uint8_t* i = (uint8_t*) ptr;
  uint8_t* j = (uint8_t*) &result;

  j[0] = i[0];
  j[1] = i[1];
  j[2] = i[2];
  j[3] = i[3];

  return result;
}

void unaligned_write_16(uint8_t* ptr, uint16_t val)
{
  uint8_t* i = (uint8_t*) ptr;
  uint8_t* j = (uint8_t*) &val;

  i[0] = j[0];
  i[1] = j[1];
}

void unaligned_write_32(uint8_t* ptr, uint32_t val)
{
  uint8_t* i = (uint8_t*) ptr;
  uint8_t* j = (uint8_t*) &val;

  i[0] = j[0];
  i[1] = j[1];
  i[2] = j[2];
  i[3] = j[3];
}
