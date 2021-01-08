#ifndef __PINS_H__
#define __PINS_H__

//! @file This file contains hardware pin definitions and macros.

#include <msp430.h>

// Pin definitions
#define P1_IPIN_IR_SENSE 0x01      // P1.0 (2)
#define P1_OPIN_IR_ACTIVE 0x02     // P1.1 (3)
#define P1_IPIN_BATT_DETECT 0x04   // P1.2 (4)
#define P1_IPIN_BUTTON 0x08        // P1.3 (5)
#define P1_OPIN_PUMP 0x10          // P1.4 (6)
#define P1_OPIN_IR_PULSE 0x20      // P1.5 (7)
#define P1_OPIN_WHITE_LED 0x40     // P1.6 (14)
#define P1_OPIN_RED_LED 0x80       // P1.7 (15)
#define ALL_P1_OUTPUTS_MASK (P1_OPIN_RED_LED | P1_OPIN_WHITE_LED | P1_OPIN_IR_PULSE | P1_OPIN_PUMP | P1_OPIN_IR_ACTIVE)

// Getter macros
#define GET_PINS(port, mask) (port & mask)
#define GET_IR_SENSE() GET_PINS(P1IN, P1_IPIN_IR_SENSE)
#define GET_BATT_DETECT() GET_PINS(P1IN, P1_IPIN_BATT_DETECT)
#define GET_IR_PULSE() GET_PINS(P1OUT, P1_OPIN_IR_PULSE)

// Setter macros
#define SET_OUTPUTS_ON(port, mask) (port |= mask)
#define SET_OUTPUTS_OFF(port, mask) (port &= ~mask)
#define SET_OUTPUTS(port, mask, on) (on ? SET_OUTPUTS_ON(port, mask) : SET_OUTPUTS_OFF(port, mask))
#define TOGGLE_OUTPUTS(port, mask) port ^= mask
#define SET_WHITE_LED(on) SET_OUTPUTS(P1OUT, P1_OPIN_WHITE_LED, on)
#define SET_RED_LED(on) SET_OUTPUTS(P1OUT, P1_OPIN_RED_LED, on)
#define TOGGLE_RED_LED() TOGGLE_OUTPUTS(P1OUT, P1_OPIN_RED_LED)
#define SET_IR_ACTIVE(on) SET_OUTPUTS(P1OUT, P1_OPIN_IR_ACTIVE, on)
#define SET_PUMP(on) SET_OUTPUTS(P1OUT, P1_OPIN_PUMP, on)
#define SET_IR_PULSE(on) SET_OUTPUTS(P1OUT, P1_OPIN_IR_PULSE, on)
// All control IO are on port 1
#define ALL_OUTPUTS_OFF() SET_OUTPUTS_OFF(P1OUT, ALL_P1_OUTPUTS_MASK)

#endif // __PINS_H__
