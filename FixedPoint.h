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

#endif /* FIXEDPOINT_H */
