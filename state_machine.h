#ifndef __STATE_MACHINE_H__
#define __STATE_MACHINE_H__

//! @file This file handles the state machine for the soap dispenser when it is powered.

#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>

// State machine modes
#define TIMER_MODE_IR_WAIT          0
#define TIMER_MODE_IR_CHECK         1
#define TIMER_MODE_DISPENSE         2
#define TIMER_MODE_PRANK_DISPENSE   3
#define TIMER_MODE_ENTER_RUN        4
#define TIMER_MODE_ENTER_PRANK_MODE 5
#define TIMER_MODE_EXIT_RUN         6
#define TIMER_MODE_COUNT            7
#define TIMER_MODE_NONE             TIMER_MODE_COUNT

//! @see definition in state_machine.c; not to be accessed externally
extern int16_t gModeCounterLookup[TIMER_MODE_COUNT];
//! @see definition in state_machine.c; not to be accessed externally
extern uint8_t gTimerMode;

//! Executes the current state; to be called from timer interrupt
//! @returns true iff we need to enter LPM4
inline bool executeState(void);

//! Sets the state machine counter values based on a VLO calibration value
//! @param[in] vloValue  Number of 8MHz CPU clock ticks per VLO ticks
inline void setStateCounters(unsigned int vloValue);

//! Transitions the state machine to the next mode
//! @param[in] mode  The next mode to transition to
inline void nextMode(uint8_t mode)
{
    CCTL0 = 0; // Make sure interrupt is disabled
    gTimerMode = mode;
    CCR0 = gModeCounterLookup[mode];
    TA0CTL |= TACLR; // Reset counter
    CCTL0 = CCIE; // Enable interrupt
}

//! @returns the currently executing mode
inline uint8_t getMode(void);

//! Stops the IR control outputs and sense interrupt
inline void outputIrStop(void);

//! Checks if battery is low
//! @returns true iff the battery is low
inline bool isBattLow(void);

//! Executes the TIMER_MODE_IR_WAIT state
inline void timerModeWait(void);

//! Executes the next IR sense loop while waiting for the hand to be removed
//! @returns true iff the hand is no longer sensed and IR output has been stopped
inline bool isHandRemoved(void);

//! Executes the TIMER_MODE_IR_CHECK state
inline void timerModeIrCheck(void);

//! Executes the TIMER_MODE_PRANK_DISPENSE state
inline void timerModePrankDispense(void);

//! Executes the TIMER_MODE_DISPENSE state
inline void timerModeDispense(void);

//! @returns true iff the state machine is currently executing the prank >:)
inline bool isExecutingPrank(void);

#endif // __STATE_MACHINE_H__
