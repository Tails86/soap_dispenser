#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>
#include "state_machine.h"
#include "pins.h"
#include "button.h"
#include "sense.h"
#include "vlo.h"

// Length of time to turn on the ON LED and OFF LED
#define ON_LED_TIME_MS 1000
#define OFF_LED_TIME_MS 1000
#define PRANK_MODE_RED_ON_LED_TIME_MS 400

// Frequency value for IR sensor
#define MAX_IR_FREQUENCY 2800
// This will compute a counter for a frequency up to IR_FREQUENCY
#define COMPUTE_IR_CHECK_COUNTER(cal) (VLO_COUNT_REL_CLOCK_FREQUENCY / 2 / MAX_IR_FREQUENCY / cal)

// Definitions of how many pulses we make for each IR check
#define IR_SENSE_PULSES 5
// Number of pulses within IR_SENSE_PULSES that must have sense before activation
#define IR_ACTIVATION_THRESHOLD 3
#if IR_ACTIVATION_THRESHOLD >= IR_SENSE_PULSES
#error "activation value must be less than IR_ACTIVATION_THRESHOLD"
#endif

// Definitions for dispense
// The number of CONSECUTIVE pulses while dispensing must not have sense before deactivation
#define IR_DEACTIVATION_THRESHOLD 200
#define MAX_DISPENSE_TIME_MS 1800
// Definitions for red LED blink to inform the user the sensor was never unblocked
#define IM_WAITING_BLINK_START_DELAY_MS 3000
#define IM_WAITING_BLINK_LENGTH_MS 100
#define IM_WAITING_BLINK_DELAY_MS 2000

// Period of time in between each check (this is also how long red LED will flash on low batt)
#define IR_WAIT_PERIOD_MS 333

// Definitions for battery detect
#define CHECK_BATT_PERIOD_S 10
#define CHECK_BATT_COUNT (int16_t)(CHECK_BATT_PERIOD_S * 1000 / IR_WAIT_PERIOD_MS)

// The amount of time to dispense into hand
#define PRANK_DISPENSE_TIME1_MS 200
// The amount of time to wait after hand is removed before moving to 3
#define PRANK_DISPENSE_TIME2_MS 300
// The maximum amount of time to dispense until hand is placed back on the sensor
#define PRANK_DISPENSE_TIME3_MS 5000
// The amount of time to dispense after the hand is removed a second time
#define PRANK_DISPENSE_TIME4_MS 10000
// The number of prank states
#define PRANK_STATE_COUNT 5

// States for blinking the LED as we're waiting for the user to remove hand
#define IM_WAITING_BLINK_START 0
#define IM_WAITING_BLINK_ON1   1
#define IM_WAITING_BLINK_OFF1  2
#define IM_WAITING_BLINK_ON2   3
#define IM_WAITING_BLINK_OFF2  4
#define IM_WAITING_BLINK_MAX   5
// The number of counts to wait before moving to next state
// These are approximations since they are computed using MAX_IR_FREQUENCY instead of actual freq.
int16_t gImWaitingCounts[5] = {
    (int16_t)(IM_WAITING_BLINK_START_DELAY_MS / 1000.0 * MAX_IR_FREQUENCY * 2),
    (int16_t)(IM_WAITING_BLINK_LENGTH_MS / 1000.0 * MAX_IR_FREQUENCY * 2),
    (int16_t)(IM_WAITING_BLINK_LENGTH_MS / 1000.0 * MAX_IR_FREQUENCY * 2),
    (int16_t)(IM_WAITING_BLINK_LENGTH_MS / 1000.0 * MAX_IR_FREQUENCY * 2),
    (int16_t)(IM_WAITING_BLINK_DELAY_MS / 1000.0 * MAX_IR_FREQUENCY * 2)
};
// The current state
uint8_t gImWaitingState = IM_WAITING_BLINK_START;

// The max timer counter for each of the modes
int16_t gModeCounterLookup[TIMER_MODE_COUNT] = {0};
// Current state machine mode
uint8_t gTimerMode = TIMER_MODE_NONE;

// The maximum dispense count as computed in calibrate()
uint16_t gMaxDispenseCount = 0;
// The dispense counts for prank mode
uint16_t gPrankDispenseCounts[4] = {0};
// Thur current prank state
uint8_t gPrankState = PRANK_STATE_COUNT;
// Current IR state count
int8_t gIrCount = 0;
// Number of pulses where hand is sensed
int16_t gSenseCount = 0;
// Current dispense time counter (used in TIMER_MODE_DISPENSE)
uint16_t gDispenseTime = 0;
// Wait counter for battery detection activation
int16_t gBattWaitCount = 0;
// Counter used to keep track of how long we have been waiting
int16_t gSensorUnblockCount = 0;

inline bool executeState(void)
{
    bool rv = false;
    switch (gTimerMode)
    {
    case TIMER_MODE_IR_WAIT:
        timerModeWait();
        break;
    case TIMER_MODE_IR_CHECK:
        timerModeIrCheck();
        break;
    case TIMER_MODE_DISPENSE:
        timerModeDispense();
        break;
    case TIMER_MODE_PRANK_DISPENSE:
        timerModePrankDispense();
        break;
    case TIMER_MODE_ENTER_RUN:
        // Enter run mode
        ALL_OUTPUTS_OFF();
        if (isPrankModeEnabled())
        {
            // This just allows the red LED to be on for a moment to signal prank mode entry
            SET_RED_LED(true);
            nextMode(TIMER_MODE_ENTER_PRANK_MODE);
        }
        else
        {
            nextMode(TIMER_MODE_IR_WAIT);
        }
        break;
    case TIMER_MODE_ENTER_PRANK_MODE:
        SET_RED_LED(false);
        nextMode(TIMER_MODE_IR_WAIT);
        break;
    case TIMER_MODE_EXIT_RUN: // Fall through
    case TIMER_MODE_NONE: // Fall through
    default:
        // Disable this interrupt, power off all outputs, and go to LPM4
        CCTL0 = 0;
        ALL_OUTPUTS_OFF();
        SET_RED_LED(false);
        rv = true; // Tell caller to enter LPM4
        break;
    }
    return rv;
}

inline void setStateCounters(unsigned int vloValue)
{
    int16_t irCheckCounter = COMPUTE_IR_CHECK_COUNTER(vloValue);
    gModeCounterLookup[TIMER_MODE_IR_WAIT] = MS_TO_VLO_COUNTS(IR_WAIT_PERIOD_MS, vloValue);
    gModeCounterLookup[TIMER_MODE_IR_CHECK] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_DISPENSE] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_PRANK_DISPENSE] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_ENTER_RUN] = MS_TO_VLO_COUNTS(ON_LED_TIME_MS, vloValue);
    gModeCounterLookup[TIMER_MODE_ENTER_PRANK_MODE] = MS_TO_VLO_COUNTS(PRANK_MODE_RED_ON_LED_TIME_MS, vloValue);
    gModeCounterLookup[TIMER_MODE_EXIT_RUN] = MS_TO_VLO_COUNTS(OFF_LED_TIME_MS, vloValue);
    gMaxDispenseCount = MS_TO_IR_CYCLES(MAX_DISPENSE_TIME_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[0] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME1_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[1] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME2_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[2] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME3_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[3] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME4_MS, vloValue, irCheckCounter);
}

inline uint8_t getMode(void)
{
    return gTimerMode;
}

inline void outputIrStop(void)
{
    SET_IR_ACTIVE(false);
    SET_IR_PULSE(false);
    SENSE_DISABLE();
}

inline bool isBattLow(void)
{
    // Set to pull-up resistor for battery detect pin
    P1OUT |= P1_IPIN_BATT_DETECT;
    P1REN |= P1_IPIN_BATT_DETECT;
    __delay_cycles(10); // make sure this takes effect
    bool battDetect = GET_BATT_DETECT();
    // Remove pull-up
    P1OUT &= ~P1_IPIN_BATT_DETECT;
    P1REN &= ~P1_IPIN_BATT_DETECT;
    return battDetect;
}

inline void timerModeWait(void)
{
    ALL_OUTPUTS_OFF();
    ++gBattWaitCount;
    if (gBattWaitCount >= CHECK_BATT_COUNT)
    {
        gBattWaitCount = 0;
        if (isBattLow())
        {
            // Turn on red LED which should be shut off in one of:
            // - Next TIMER_MODE_IR_WAIT entry on call to ALL_OUTPUTS_OFF()
            // - In TIMER_MODE_IR_CHECK when dispense entered
            // - When device is powered off
            SET_RED_LED(true);
        }
    }
    // Go to check mode
    SET_IR_ACTIVE(true);
    gIrCount = 0;
    gSenseCount = 0;
    // Turn interrupt for sense high to low
    resetSense();
    SENSE_ENABLE();
    // Waiting a moment to go into check mode (instead of just dropping right into it)
    // This will allow the IR active line to go high before first entry
    nextMode(TIMER_MODE_IR_CHECK);
}

inline bool isHandRemoved(void)
{
    // Are we done sensing for the hand yet? (wait for hand to be removed)
    if (gSenseCount < IR_DEACTIVATION_THRESHOLD)
    {
        if (GET_IR_PULSE())
        {
            // Currently high; check if hand is no longer sensed then go low
            if (!isSensed())
            {
                // Hand is not sensed
                ++gSenseCount;
                if (gSenseCount >= IR_DEACTIVATION_THRESHOLD)
                {
                    // Passed threshold; we no longer need to wait
                    outputIrStop();
                    gSenseCount = 0x7FFF; // Max int16
                }
            }
            else
            {
                // Reset sense count to enforce that we get consecutive samples
                gSenseCount = 0;
            }
            // Pulse goes low
            SET_IR_PULSE(false);
        }
        else
        {
            // Sense should happen about 30 us after false->true transition if hand is there
            resetSense();
            // Pulse goes high
            SET_IR_PULSE(true);
        }
    }
    return (gSenseCount >= IR_DEACTIVATION_THRESHOLD);
}

inline void timerModeIrCheck(void)
{
    if (GET_IR_PULSE())
    {
        // Currently high; check if hand is sensed or if we are done waiting
        ++gIrCount;
        if (isSensed())
        {
            // Hand is sensed
            ++gSenseCount;
            if (gSenseCount >= IR_ACTIVATION_THRESHOLD)
            {
                // Passed threshold; dispense and wait for hand to clear
                SET_RED_LED(false); // In case we were blinking for low batt from check in TIMER_MODE_IR_WAIT
                SET_WHITE_LED(true);
                gDispenseTime = 0;
                gSenseCount = 0;
                gSensorUnblockCount = 0;
                resetSense();
                gImWaitingState = IM_WAITING_BLINK_START;
                SET_PUMP(true);
                if (isPrankModeEnabled())
                {
                    // Enter prank mode >:)
                    gPrankState = 0;
                    nextMode(TIMER_MODE_PRANK_DISPENSE);
                }
                else
                {
                    nextMode(TIMER_MODE_DISPENSE);
                }
            }
        }
        else if (gIrCount >= IR_SENSE_PULSES || gSenseCount + IR_SENSE_PULSES - gIrCount < IR_ACTIVATION_THRESHOLD)
        {
            // All pulses were made without sensing hand or there is no way to reach the threshold with subsequent pulses
            // Stop and go to wait state
            outputIrStop();
            nextMode(TIMER_MODE_IR_WAIT);
        }
        // Pulse goes low
        SET_IR_PULSE(false);
    }
    else
    {
        // Sense should happen about 30 us after false->true transition if hand is there
        resetSense();
        // Pulse goes high
        SET_IR_PULSE(true);
    }
}

inline void timerModePrankDispense(void)
{
    static uint8_t prankLoop = 0;
    ++gDispenseTime;
    switch (gPrankState)
    {
    case 0:
        // Dispense for a short time and wait for hand to be removed
        if (gDispenseTime >= gPrankDispenseCounts[0])
        {
            SET_PUMP(false);
            SET_WHITE_LED(false);
        }
        if (isHandRemoved())
        {
            SET_IR_ACTIVE(false);
            ++gPrankState;
            gDispenseTime = 0;
            prankLoop = 0;
        }
        break;
    case 1:
        // Wait for a short period
        if (gDispenseTime >= gPrankDispenseCounts[1])
        {
            // Turn on the pump and move to the next state
            SET_PUMP(true);
            SET_WHITE_LED(true);
            SET_IR_ACTIVE(true);
            SET_IR_PULSE(false);
            resetSense();
            gSenseCount = 0;
            SENSE_ENABLE();
            gDispenseTime = 0;
            ++gPrankState;
        }
        break;
    case 2:
        // Dispense either for the set period or until hand is placed back under the sensor
        if (gDispenseTime < gPrankDispenseCounts[2])
        {
            if (GET_IR_PULSE())
            {
                // Currently high; check if hand is sensed or if we are done waiting
                if (isSensed())
                {
                    // Hand is sensed
                    ++gSenseCount;
                    if (gSenseCount >= IR_ACTIVATION_THRESHOLD)
                    {
                        // Passed threshold; stop the pump >:)
                        SET_PUMP(false);
                        SET_WHITE_LED(false);
                        // Go to next state
                        ++gPrankState;
                        resetSense();
                        gDispenseTime = 0;
                        gSenseCount = 0;
                    }
                }
                // Pulse goes low
                SET_IR_PULSE(false);
            }
            else
            {
                // Sense should happen about 30 us after false->true transition if hand is there
                resetSense();
                // Pulse goes high
                SET_IR_PULSE(true);
            }

        }
        else
        {
            // Just go back to waiting
            SET_PUMP(false);
            SET_WHITE_LED(false);
            outputIrStop();
            nextMode(TIMER_MODE_IR_WAIT);
        }
        break;
    case 3:
        SET_PUMP(false);
        SET_WHITE_LED(false);
        // Wait indefinitely until hand is removed
        if (isHandRemoved())
        {
            // Turn pump back on
            SET_PUMP(true);
            SET_WHITE_LED(true);
            // Get ready to wait for hand released
            SET_IR_ACTIVE(true);
            SET_IR_PULSE(false);
            resetSense();
            gSenseCount = 0;
            SENSE_ENABLE();
            // Go to next state or back to 1
            gDispenseTime = 0;
            if (++prankLoop >= 3)
            {
                ++gPrankState;
            }
            else
            {
                gPrankState = 1;
            }
        }
        break;
    case 4:
        if (gDispenseTime >= gPrankDispenseCounts[3] && isHandRemoved())
        {
            // We are done here
            ++gPrankState;
            SET_PUMP(false);
            SET_WHITE_LED(false);
            outputIrStop();
            nextMode(TIMER_MODE_IR_WAIT);
        }
        break;
    default:
        ALL_OUTPUTS_OFF();
        nextMode(TIMER_MODE_IR_WAIT);
        break;
    }
}

inline void timerModeDispense(void)
{
    // Is the hand now removed from the sensor?
    if (isHandRemoved())
    {
        // We are done dispensing
        ALL_OUTPUTS_OFF();
        nextMode(TIMER_MODE_IR_WAIT);
        // Force battery check on IR wait entry
        gBattWaitCount = CHECK_BATT_COUNT;
    }
    else if (gDispenseTime < gMaxDispenseCount)
    {
        ++gDispenseTime;
    }
    // Otherwise, we reached the maximum 1.8s threshold?
    else
    {
        // Turn off the pump, but we aren't done "dispensing" until hand is removed.
        SET_PUMP(false);
        SET_WHITE_LED(false);
        // Blink red LED if hand continues to be sensed for an extended period of time
        if (gSensorUnblockCount >= gImWaitingCounts[gImWaitingState])
        {
            // go to next state
            ++gImWaitingState;
            if (gImWaitingState >= IM_WAITING_BLINK_MAX)
            {
                gImWaitingState = IM_WAITING_BLINK_ON1;
            }
            SET_RED_LED(gImWaitingState == IM_WAITING_BLINK_ON1 || gImWaitingState == IM_WAITING_BLINK_ON2);
            gSensorUnblockCount = 0;
        }
        ++gSensorUnblockCount;
    }
}

inline bool isExecutingPrank(void)
{
    return gPrankState < PRANK_STATE_COUNT;
}
