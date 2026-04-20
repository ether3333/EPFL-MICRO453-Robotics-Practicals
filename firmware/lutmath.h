#ifndef __LUTMATH_H
#define __LUTMATH_H

/**
 * \file   lutmath.h
 * \brief  Trigonometric functions (sine, cosine) based on lookup tables
 * \author Alessandro Crespi
 * \date   July 2009
 */

/// Lookup table based sine function
float sinlut(float in) __attribute__((const));

/// Lookup table based cosine function
float coslut(float in) __attribute__((const));

#endif // __LUTMATH_H
