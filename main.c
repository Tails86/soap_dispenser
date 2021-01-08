#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
#include "pins.h"

/**
 * main.c
 */
int main(void)
{
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
    BCSCTL1 = CALBC1_1MHZ;      // Set range   DCOCTL = CALDCO_1MHZ;
    BCSCTL2 &= ~(DIVS_3);       // SMCLK = DCO = 1MHz
    // Set ACLK to internal VLO (4-20 kHz, typ=12kHz)
    BCSCTL3 &= ~LFXT1S0;
    BCSCTL3 |= LFXT1S1;
    BCSCTL1 &= ~XTS;
    // All P1 setup for GPIO
    P1SEL = 0;
    P1SEL2 = 0;
    // Nothing on P2
    P2SEL = 0;
    P2SEL2 = 0;
    // Setup P1 outputs
    P1OUT = 0;
    P1DIR = ALL_P1_OUTPUTS_MASK;
    // Button interrupt on low to high; IR sense from high to low but not enabled here
    P1IE = P1_IPIN_BUTTON;
    P1IES = P1_IPIN_IR_SENSE;
    // Timer 0 control
    CCTL0 = 0;                  // Timer interrupt disabled
    TA0CTL = TASSEL_1 | MC_1;   // ACLK, up mode


    while(1)
    {
        // Finally, enter LPM4 and enable interrupts
        // (initialized as powered off, waiting for first button press)
        __bis_SR_register(LPM4_bits + GIE);
        // Should not reach here, but just in case...
        ALL_OUTPUTS_OFF();
    }

    // Unreachable
    return 0;
}


