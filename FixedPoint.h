/**
  FixedPoint.h - Fixed point math

  Copyright 2017 Costin STROIE <costinstroie@eridu.eu.org>

  This file is part of RoboMitzy.
*/

#ifndef FIXEDPOINT_H
#define FIXEDPOINT_H

typedef int32_t fp_t;
typedef int64_t fpd_t;
#define FP_BITS       32
#define FP_FBITS      8
#define FP_WBITS      (FP_BITS - FP_FBITS)
#define FP_FMASK      (((fp_t)1 << FP_FBITS) - 1)
#define FP_ONE        ((fp_t)((fp_t)1 << FP_FBITS))
#define FP_HALF       (FP_ONE >> 1)

// Use symmetric ranges, use the 0x80..00 values as exceptions
#define MAX32     0x7FFFFFFFL
#define MIN32    -MAX32
#define MAX24     0x7FFFFFL
#define MIN24    -MAX24
#define MAX16     0x7FFF
#define MIN16    -MAX16
#define MAX8      0x7F
#define MIN8     -MAX8

#endif /* FIXEDPOINT_H */
