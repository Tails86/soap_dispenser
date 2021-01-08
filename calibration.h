#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

//! @file This file handles system calibration logic.

#include <msp430.h>
#include <stdbool.h>
#include "state_machine.h"
#include "vlo.h"

//! Calibrates the system using the CPU clock
inline bool calibrate(void)
{
    bool rv = false;
    // TI_measureVLO messes with TA0CTL, so save it
    uint16_t origTa0ctl = TA0CTL;
    // These are some pretty intensive calculations, but they only happen during
    // power on! I'm not going to worry about moving these operations to RAM.
    unsigned int vloValue = TI_measureVLO();
    CCTL0 = 0;                  // Timer interrupt disabled
    TA0CTL = origTa0ctl;        // Restore TA0CTL
    // Validate the measurement
    if (vloValue >= MIN_VLO_COUNT && vloValue <= MAX_VLO_COUNT)
    {
        setStateCounters(vloValue);
        rv = true;
    }
    return rv;
}

#endif // __CALIBRATION_H__
