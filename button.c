#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "state_machine.h"
#include "pins.h"
#include "button.h"
#include "calibration.h"

// Defines how many consecutive power ons are necessary to enable prank mode
#define PRANK_MODE_POWER_ON_COUNT 3

// Current power state (true=ON; false=OFF)
bool gPowerState = false;
// Counts the number of consecutive power on cycles
int16_t gPowerOnCount = 0;
// Prank mode enabled once the above reaches PRANK_MODE_POWER_ON_COUNT
bool gPrankModeEnabled = false;

inline bool handleButtonPress(void)
{
    bool rv = false;
    // Don't allow power control while on and running the prank
    if (!gPowerState || !gPrankModeEnabled || !isExecutingPrank())
    {
        if (gPowerState)
        {
            // Go from on to off
            if (getMode() != TIMER_MODE_ENTER_RUN)
            {
                gPowerOnCount = 0;
            }
            gPrankModeEnabled = false;
            ALL_OUTPUTS_OFF();
            SET_RED_LED(true);
            nextMode(TIMER_MODE_EXIT_RUN);
            gPowerState = false; // We are now off
        }
        else
        {
            // Go from off to on
            if (getMode() == TIMER_MODE_EXIT_RUN)
            {
                ++gPowerOnCount;
                gPrankModeEnabled = (gPowerOnCount == PRANK_MODE_POWER_ON_COUNT);
            }
            else
            {
                gPowerOnCount = 1;
            }
            ALL_OUTPUTS_OFF();
            if (calibrate()) // Calibrate timer
            {
                SET_WHITE_LED(true);
                nextMode(TIMER_MODE_ENTER_RUN);
                gPowerState = true; // We are now on
                rv = true; // Tell the caller that we need to enter LPM3
            }
            else
            {
                // Calibration failed which means there is no way to enter the state machine
                SET_RED_LED(true);
                uint8_t i = 9;
                for (; i > 0; --i)
                {
                    __delay_cycles(200000);
                    TOGGLE_RED_LED();
                }
                ALL_OUTPUTS_OFF();
                // Remain powered off
            }
        }
    }
    return rv;
}

inline bool isPrankModeEnabled(void)
{
    return gPrankModeEnabled;
}
