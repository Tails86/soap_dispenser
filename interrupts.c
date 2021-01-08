#include <msp430.h>
#include "pins.h"
#include "state_machine.h"
#include "button.h"
#include "sense.h"

// Enable or disable oscillator on exit of interrupt (the difference between LPM3 and LPM4)
#define ENABLE_OSCILLATOR_ON_EXIT() __bic_SR_register_on_exit(OSCOFF)
#define DISABLE_OSCILLATOR_ON_EXIT() __bis_SR_register_on_exit(OSCOFF)

//Timer ISR
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
    if (executeState())
    {
        DISABLE_OSCILLATOR_ON_EXIT(); // Essentially moving from LPM3 to LPM4
        // On exit: the only interrupt accessible is for button press
    }
}

// Button ISR
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{
  if (P1IFG & P1_IPIN_BUTTON)
  {
      // Button was pressed
      // Note: debounce is not needed because that is handled by capacitive button sensor chip

      P1IFG &= ~P1_IPIN_BUTTON; // Clear interrupt flag
      if (handleButtonPress())
      {
          ENABLE_OSCILLATOR_ON_EXIT(); // Essentially moving from LPM4 to LPM3
      }
  }
  else if (P1IFG & P1_IPIN_IR_SENSE)
  {
      // Hand was sensed

      P1IFG &= ~P1_IPIN_IR_SENSE; // Clear interrupt flag
      handleSense();
  }
}
