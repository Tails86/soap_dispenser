#ifndef __SENSE_H__
#define __SENSE_H__

//! @file This file handles the IR sensing hardware.

#include <stdbool.h>

// Enables or disables the sense interrupt
#define SENSE_ENABLE() P1IE |= P1_IPIN_IR_SENSE
#define SENSE_DISABLE() P1IE &= ~P1_IPIN_IR_SENSE

//! Handles sense signal; to be called from an interrupt when hand is sensed
inline void handleSense(void);

//! @returns true when hand was sensed since resetSense() was last called
inline bool isSensed(void);

//! Resets the sense flag
inline void resetSense(void);

#endif // __SENSE_H__
