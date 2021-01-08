#include <msp430.h>
#include <stdbool.h>
#include "sense.h"

// Set by hand sense input interrupt
bool gHandSensed = false;

inline void handleSense(void)
{
    gHandSensed = true;
}

inline bool isSensed(void)
{
    return gHandSensed;
}

inline void resetSense(void)
{
    gHandSensed = false;
}
