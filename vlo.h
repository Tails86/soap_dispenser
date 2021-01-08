#ifndef __VLO_H__
#define __VLO_H__

//! @file This file contains functionality, calibration macros, and constants relating to the VLO.

#include <msp430.h>
// VLO library found on TI website
// Application Report: https://www.ti.com/lit/an/slaa340a/slaa340a.pdf
// Library download location: http://www.ti.com/lit/zip/slaa340
// Refer to the copyright notice in VLO_Library.asm.
// This library allows me to use the Very Low Oscillator (VLO) with greater timing precision.
#include "VLO_Library.h"

// Min/max VLO frequencies from specs
#define MIN_VLO_FREQUENCY 4000
#define MAX_VLO_FREQUENCY 20000
// The clock frequency the VLO count is relative to (coded into VLO_Library.asm)
#define VLO_COUNT_REL_CLOCK_FREQUENCY 8000000
// Computed min and max VLO counts
#define MIN_VLO_COUNT (VLO_COUNT_REL_CLOCK_FREQUENCY / MAX_VLO_FREQUENCY) // 400
#define MAX_VLO_COUNT (VLO_COUNT_REL_CLOCK_FREQUENCY / MIN_VLO_FREQUENCY) // 2000

// Converts milliseconds to clock counts for CCR0
#define MS_TO_VLO_COUNTS(ms, cal) (VLO_COUNT_REL_CLOCK_FREQUENCY / 1000 * ms / cal - 1)
// Converts milliseconds to IR cycles
#define MS_TO_IR_CYCLES(ms, vloValue, irCheckCounter) (ms * (VLO_COUNT_REL_CLOCK_FREQUENCY / 1000) / vloValue / (irCheckCounter + 1))

#endif // __VLO_H__
