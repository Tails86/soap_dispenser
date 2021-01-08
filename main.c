#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>
// VLO library found on TI website
// Application Report: https://www.ti.com/lit/an/slaa340a/slaa340a.pdf
// Library download location: http://www.ti.com/lit/zip/slaa340
// Refer to the copyright notice in VLO_Library.asm.
// This library allows me to use the Very Low Oscillator (VLO) with greater timing precision.
#include "VLO_Library.h"

inline void calibrate();
inline void nextMode(uint8_t mode);
inline void outputIrStop();
inline bool isBattLow();
inline void timerModeWait();
inline bool isHandRemoved();
inline void timerModeIrCheck();
inline void timerModePrankDispense();
inline void timerModeDispense();

// Pin definitions
#define P1_OPIN_IR_ACTIVE 0x02
#define P1_OPIN_PUMP 0x10
#define P1_OPIN_IR_PULSE 0x20
#define P1_OPIN_WHITE_LED 0x40
#define P1_OPIN_RED_LED 0x80
#define ALL_P1_OUTPUTS_MASK (P1_OPIN_RED_LED | P1_OPIN_WHITE_LED | P1_OPIN_IR_PULSE | P1_OPIN_PUMP | P1_OPIN_IR_ACTIVE)
#define P1_IPIN_IR_SENSE 0x01
#define P1_IPIN_BATT_DETECT 0x04
#define P1_IPIN_BUTTON 0x08

// Getter macros
#define GET_PINS(port, mask) (port & mask)
#define GET_IR_SENSE() GET_PINS(P1IN, P1_IPIN_IR_SENSE)
#define GET_BATT_DETECT() GET_PINS(P1IN, P1_IPIN_BATT_DETECT)
#define GET_IR_PULSE() GET_PINS(P1OUT, P1_OPIN_IR_PULSE)

// Setter macros
#define SET_OUTPUT_ON(port, mask) (port |= mask)
#define SET_OUTPUT_OFF(port, mask) (port &= ~mask)
#define SET_OUTPUT(port, mask, on) (on ? SET_OUTPUT_ON(port, mask) : SET_OUTPUT_OFF(port, mask))
#define SET_WHITE_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_WHITE_LED, on)
#define SET_RED_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_RED_LED, on)
#define SET_IR_ACTIVE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_ACTIVE, on)
#define SET_PUMP(on) SET_OUTPUT(P1OUT, P1_OPIN_PUMP, on)
#define SET_IR_PULSE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_PULSE, on)
// All control IO are on port 1
#define ALL_OUTPUTS_OFF() SET_OUTPUT_OFF(P1OUT, ALL_P1_OUTPUTS_MASK)

// Min/max VLO frequencies from specs
#define MIN_VLO_FREQUENCY 4000
#define MAX_VLO_FREQUENCY 20000
// The clock frequency the VLO count is relative to (coded into VLO_Library.asm)
#define VLO_COUNT_REL_CLOCK_FREQUENCY 8000000
// Computed min and max VLO counts
#define MIN_VLO_COUNT (VLO_COUNT_REL_CLOCK_FREQUENCY / MAX_VLO_FREQUENCY) // 400
#define MAX_VLO_COUNT (VLO_COUNT_REL_CLOCK_FREQUENCY / MIN_VLO_FREQUENCY) // 2000

// Enable or disable oscillator on exit of interrupt (the difference between LPM3 and LPM4)
#define ENABLE_OSCILLATOR_ON_EXIT() __bic_SR_register_on_exit(OSCOFF)
#define DISABLE_OSCILLATOR_ON_EXIT() __bis_SR_register_on_exit(OSCOFF)
// Converts milliseconds to clock counts for CCR0
#define MS_TO_COUNTS(ms, cal) (VLO_COUNT_REL_CLOCK_FREQUENCY / 1000 * ms / cal - 1)
// Converts milliseconds to IR cycles
#define MS_TO_IR_CYCLES(ms, vloValue, irCheckCounter) (ms * (VLO_COUNT_REL_CLOCK_FREQUENCY / 1000) / vloValue / (irCheckCounter + 1))

// Length of time to turn on the ON LED and OFF LED
#define ON_LED_TIME_MS 1000
#define OFF_LED_TIME_MS 1000

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

// Defines how many consecutive power ons are necessary to enable prank mode
#define PRANK_MODE_POWER_ON_COUNT 3

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

// State machine modes
#define TIMER_MODE_IR_WAIT        0
#define TIMER_MODE_IR_CHECK       1
#define TIMER_MODE_DISPENSE       2
#define TIMER_MODE_PRANK_DISPENSE 3
#define TIMER_MODE_ENTER_RUN      4
#define TIMER_MODE_EXIT_RUN       5
#define TIMER_MODE_NONE           6
// The max timer counter for each of the above modes
int16_t gModeCounterLookup[6] = {0};
// Current state machine mode
int8_t gTimerMode = TIMER_MODE_NONE;

// Current power state (true=ON; false=OFF)
bool gPowerState = false;
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
// Set by hand sense input interrupt
bool gHandSensed = false;
// Current dispense time counter (used in TIMER_MODE_DISPENSE)
uint16_t gDispenseTime = 0;
// Wait counter for battery detection activation
int16_t gBattWaitCount = 0;
// Counter used to keep track of how long we have been waiting
int16_t gSensorUnblockCount = 0;
// Counts the number of consecutive power on cycles
int16_t gPowerOnCount = 0;
// Prank mode enabled once the above reaches PRANK_MODE_POWER_ON_COUNT
bool gPrankModeEnabled = false;
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

inline void calibrate()
{
    // This is some pretty intensive calculations for a microcontroller, but they only happen during
    // power on!
    unsigned int vloValue = TI_measureVLO();
    int16_t irCheckCounter = COMPUTE_IR_CHECK_COUNTER(vloValue);
    gModeCounterLookup[TIMER_MODE_IR_WAIT] = MS_TO_COUNTS(IR_WAIT_PERIOD_MS, vloValue);
    gModeCounterLookup[TIMER_MODE_IR_CHECK] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_DISPENSE] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_PRANK_DISPENSE] = irCheckCounter;
    gModeCounterLookup[TIMER_MODE_ENTER_RUN] = MS_TO_COUNTS(ON_LED_TIME_MS, vloValue);
    gModeCounterLookup[TIMER_MODE_EXIT_RUN] = MS_TO_COUNTS(OFF_LED_TIME_MS, vloValue);
    gMaxDispenseCount = MS_TO_IR_CYCLES(MAX_DISPENSE_TIME_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[0] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME1_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[1] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME2_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[2] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME3_MS, vloValue, irCheckCounter);
    gPrankDispenseCounts[3] = MS_TO_IR_CYCLES(PRANK_DISPENSE_TIME4_MS, vloValue, irCheckCounter);
    // Timer 0 control
    CCTL0 = 0;                  // Timer interrupt disabled
    TA0CTL = TASSEL_1 | MC_1;   // ACLK, up mode
}

inline void nextMode(uint8_t mode)
{
    CCTL0 = 0; // Make sure interrupt is disabled
    gTimerMode = mode;
    CCR0 = gModeCounterLookup[mode];
    TA0CTL |= TACLR; // Reset counter
    CCTL0 = CCIE; // Enable interrupt
}

inline void outputIrStop()
{
    SET_IR_ACTIVE(false);
    SET_IR_PULSE(false);
    P1IE &= ~P1_IPIN_IR_SENSE;
}

inline bool isBattLow()
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

inline void timerModeWait()
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
    gHandSensed = false;
    P1IE |= P1_IPIN_IR_SENSE;
    // Waiting a moment to go into check mode (instead of just dropping right into it)
    // This will allow the IR active line to go high before first entry
    nextMode(TIMER_MODE_IR_CHECK);
}

inline bool isHandRemoved()
{
    // Are we done sensing for the hand yet? (wait for hand to be removed)
    if (gSenseCount < IR_DEACTIVATION_THRESHOLD)
    {
        if (GET_IR_PULSE())
        {
            // Currently high; check if hand is no longer sensed then go low
            if (!gHandSensed)
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
            gHandSensed = false;
            // Pulse goes high
            SET_IR_PULSE(true);
        }
    }
    return (gSenseCount >= IR_DEACTIVATION_THRESHOLD);
}

inline void timerModeIrCheck()
{
    if (GET_IR_PULSE())
    {
        // Currently high; check if hand is sensed or if we are done waiting
        ++gIrCount;
        if (gHandSensed)
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
                gHandSensed = false;
                gImWaitingState = IM_WAITING_BLINK_START;
                SET_PUMP(true);
                if (gPrankModeEnabled)
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
        gHandSensed = false;
        // Pulse goes high
        SET_IR_PULSE(true);
    }
}

inline void timerModePrankDispense()
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
            gHandSensed = false;
            gSenseCount = 0;
            P1IE |= P1_IPIN_IR_SENSE;
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
                if (gHandSensed)
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
                        gHandSensed = false;
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
                gHandSensed = false;
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
            gHandSensed = false;
            gSenseCount = 0;
            P1IE |= P1_IPIN_IR_SENSE;
            // Go to next state or back to 1
            gDispenseTime = 0;
            if (prankLoop++ >= 3)
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

inline void timerModeDispense()
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

//Timer ISR
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
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
        nextMode(TIMER_MODE_IR_WAIT);
        if (gPrankModeEnabled)
        {
            SET_RED_LED(true);
            __delay_cycles(100000);
            SET_RED_LED(false);
        }
        break;
    case TIMER_MODE_EXIT_RUN: // Fall through
    case TIMER_MODE_NONE: // Fall through
    default:
        // Disable this interrupt, power off all outputs, and go to LPM4
        CCTL0 = 0;
        ALL_OUTPUTS_OFF();
        SET_RED_LED(false);
        __delay_cycles(5); // I think this helps to make sure outputs are set before going into LPM4
        DISABLE_OSCILLATOR_ON_EXIT(); // Essentially moving from LPM3 to LPM4
        // On exit: the only interrupt accessible is for button press
        break;
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
      // Don't allow power control while on and running the prank
      if (!gPowerState || !gPrankModeEnabled || gPrankState >= PRANK_STATE_COUNT)
      {
          if (gPowerState)
          {
              // Go from on to off
              if (gTimerMode != TIMER_MODE_ENTER_RUN)
              {
                  gPowerOnCount = 0;
              }
              gPrankModeEnabled = false;
              ALL_OUTPUTS_OFF();
              SET_RED_LED(true);
              nextMode(TIMER_MODE_EXIT_RUN);
          }
          else
          {
              // Go from off to on
              if (gTimerMode == TIMER_MODE_EXIT_RUN)
              {
                  ++gPowerOnCount;
                  gPrankModeEnabled = (gPowerOnCount == PRANK_MODE_POWER_ON_COUNT);
              }
              else
              {
                  gPowerOnCount = 1;
              }
              ALL_OUTPUTS_OFF();
              SET_WHITE_LED(true);
              calibrate(); // Calibrate timer
              nextMode(TIMER_MODE_ENTER_RUN);
              ENABLE_OSCILLATOR_ON_EXIT(); // Essentially moving from LPM4 to LPM3
          }
          // Toggle to new state
          gPowerState = !gPowerState;
      }
  }
  else if (P1IFG & P1_IPIN_IR_SENSE)
  {
      // Hand was sensed

      P1IFG &= ~P1_IPIN_IR_SENSE; // Clear interrupt flag
      gHandSensed = true;
  }
}

