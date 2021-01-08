#ifndef __BUTTON_H__
#define __BUTTON_H__

//! @file This file contains the logic to handle button presses for power and to enter prank mode.

#include <stdbool.h>

//! Handles button press signal; to be called from an interrupt when the button is pressed
//! @returns true iff we need to exit LPM4 into LPM3
inline bool handleButtonPress(void);

//! @returns true iff on and prank mode is enabled
inline bool isPrankModeEnabled(void);

#endif // __BUTTON_H__
