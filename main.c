#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>

inline void nextMode(int8_t mode);
inline void outputIrStop();
inline bool isBattLow();
inline void timerModeWait();
inline void timerModeIrCheck();
inline void prankDispense();
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
#define SET_OUTPUT(port, mask, on) (on ? (port |= mask) : (port &= ~mask))
#define TOGGLE_OUTPUT(port, mask) (port ^= mask)
#define SET_WHITE_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_WHITE_LED, on)
#define SET_RED_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_RED_LED, on)
#define SET_IR_ACTIVE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_ACTIVE, on)
#define SET_PUMP(on) SET_OUTPUT(P1OUT, P1_OPIN_PUMP, on)
#define SET_IR_PULSE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_PULSE, on)
#define TOGGLE_IR_PULSE() TOGGLE_OUTPUT(P1OUT, P1_OPIN_IR_PULSE)
// All control IO are on port 1
#define ALL_OUTPUTS_OFF() P1OUT &= ~ALL_P1_OUTPUTS_MASK

// External crystal frequency
#define ACLK_FREQ 32768
// Enable or disable oscillator on exit of interrupt (the difference between LPM3 and LPM4)
#define ENABLE_OSCILLATOR_ON_EXIT() __bic_SR_register_on_exit(OSCOFF)
#define DISABLE_OSCILLATOR_ON_EXIT() __bis_SR_register_on_exit(OSCOFF)
// Converts milliseconds to clock counts for CCR0 (float math only done by precompiler)
#define MS_TO_COUNTS(ms) ((ms / 1000.0) * ACLK_FREQ - 1)

// Length of time to turn on the ON LED and OFF LED
#define ON_LED_TIME_MS 1000
#define OFF_LED_TIME_MS 1000

// Frequency and counter value for IR sensor
#define IR_FREQUNCY 2340
#define IR_CHECK_COUNTER ACLK_FREQ / IR_FREQUNCY / 2 - 1

// Definitions of how many pulses we make for each IR check
#define IR_SENSE_PULSES 5
// Number of pulses within IR_SENSE_PULSES that must have sense before activation
#define IR_ACTIVATION_THRESHOLD 3
#define MAX_IR_SENSE_TIME_MS (IR_SENSE_PULSES * 1000 / IR_FREQUNCY)
#if IR_ACTIVATION_THRESHOLD >= IR_SENSE_PULSES
#error "activation value must be less than IR_ACTIVATION_THRESHOLD"
#endif

// Definitions for dispense
// The number of CONSECUTIVE pulses while dispensing must not have sense before deactivation
#define IR_DEACTIVATION_THRESHOLD 200
#define MIN_DISPENSE_TIME_S 0.1
#define MAX_DISPENSE_TIME_S 1.8
// Need to multiply this by 2 because the counts are incremented on each state transition
#define MIN_DISPENSE_COUNT (int16_t)(MIN_DISPENSE_TIME_S * IR_FREQUNCY * 2)
#define MAX_DISPENSE_COUNT (int16_t)(MAX_DISPENSE_TIME_S * IR_FREQUNCY * 2)
// Definitions for red LED blink to inform the user the sensor was never unblocked
#define IM_WAITING_BLINK_START_DELAY_MS 3000
#define IM_WAITING_BLINK_LENGTH_MS 100
#define IM_WAITING_BLINK_DELAY_MS 3000
#define IM_WAITING_BLINK_START_COUNT (int16_t)(IM_WAITING_BLINK_START_DELAY_MS / 1000.0 * IR_FREQUNCY * 2)
#define IM_WAITING_BLINK_LENGTH_COUNT (int16_t)(IM_WAITING_BLINK_START_COUNT + (IM_WAITING_BLINK_LENGTH_MS / 1000.0 * IR_FREQUNCY * 2))
#define IM_WAITING_BLINK_DELAY_COUNT (int16_t)(IM_WAITING_BLINK_LENGTH_COUNT + (IM_WAITING_BLINK_DELAY_MS / 1000.0 * IR_FREQUNCY * 2))

// Period of time in between each check (this is also how long red LED will flash on low batt)
#define IR_WAIT_PERIOD_MS 400

// Definitions for battery detect
#define CHECK_BATT_PERIOD_S 10
#define CHECK_BATT_COUNT (int16_t)(CHECK_BATT_PERIOD_S * 1000 / (IR_WAIT_PERIOD_MS + MAX_IR_SENSE_TIME_MS))

// Defines how many consecutive power ons are necessary to enable prank mode
#define PRANK_MODE_POWER_ON_COUNT 3

// State machine modes
#define TIMER_MODE_IR_WAIT      0
#define TIMER_MODE_IR_CHECK     1
#define TIMER_MODE_DISPENSE     2
#define TIMER_MODE_ENTER_RUN    3
#define TIMER_MODE_EXIT_RUN     4
#define TIMER_MODE_NONE         5
// The max timer counter for each of the above modes
int16_t gModeCounterLookup[5] = {
    MS_TO_COUNTS(IR_WAIT_PERIOD_MS),
    IR_CHECK_COUNTER,
    IR_CHECK_COUNTER,
    MS_TO_COUNTS(ON_LED_TIME_MS),
    MS_TO_COUNTS(OFF_LED_TIME_MS)
};

// Current state machine mode
int8_t gTimerMode = TIMER_MODE_NONE;

// Current power state (true=ON; false=OFF)
bool gPowerState = false;
// Current IR state count
int8_t gIrCount = 0;
// Number of pulses where hand is sensed
int16_t gSenseCount = 0;
// Set by hand sense input interrupt
bool gHandSensed = false;
// Current dispense time counter (used in TIMER_MODE_DISPENSE)
int16_t gDispenseTime = 0;
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
    // All P1 setup for GPIO
    P1SEL = 0;
    P1SEL2 = 0;
    // Setup P1 outputs
    P1OUT = 0;
    P1DIR = ALL_P1_OUTPUTS_MASK;
    // Timer 0 control
    CCTL0 = 0;                  // Timer interrupt initially disabled
    CCR0 = 7 - 1;
    TA0CTL = TASSEL_1 | MC_1;   // ACLK, up mode
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

inline void nextMode(int8_t mode)
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
            gHandSensed = false;
            if (gSenseCount >= IR_ACTIVATION_THRESHOLD)
            {
                // Passed threshold; dispense and wait for hand to clear
                SET_RED_LED(false); // In case we were blinking for low batt from check in TIMER_MODE_IR_WAIT
                SET_WHITE_LED(true);
                gDispenseTime = 0;
                SET_IR_ACTIVE(false);
                SET_PUMP(true);
                if (gPrankModeEnabled)
                {
                    // Do prank mode dispense right in line (no way to power off)
                    prankDispense();
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
    }
    TOGGLE_IR_PULSE();
}

inline void prankDispense()
{
    SET_PUMP(true);
    SET_WHITE_LED(true);
    __delay_cycles(1000000);
    SET_PUMP(false);
    SET_WHITE_LED(false);
    __delay_cycles(3000000);
    SET_PUMP(true);
    SET_WHITE_LED(true);
    __delay_cycles(10000);
    SET_PUMP(false);
    __delay_cycles(50000);
    SET_PUMP(true);
    __delay_cycles(50000);
    SET_WHITE_LED(true);
    __delay_cycles(30000);
    SET_WHITE_LED(false);
    __delay_cycles(30000);
    SET_WHITE_LED(true);
    SET_PUMP(false);
    __delay_cycles(3000000);
    SET_PUMP(true);
    __delay_cycles(3000000);
    SET_PUMP(false);
    SET_WHITE_LED(false);
}

inline void timerModeDispense()
{
    ++gDispenseTime;
    // Have we reached the minimum 0.9s threshold?
    if (gDispenseTime == MIN_DISPENSE_COUNT)
    {
        // Activate IR and reset counts/flag because we will begin to pulse it on the next cycle
        SET_IR_ACTIVE(true);
        SET_IR_PULSE(false);
        gSenseCount = 0;
        gSensorUnblockCount = 0;
        gHandSensed = false;
    }
    else if (gDispenseTime > MIN_DISPENSE_COUNT)
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
                    gHandSensed = false;
                }
            }
            TOGGLE_IR_PULSE();
        }
        // Is the hand now removed from the sensor?
        if (gSenseCount >= IR_DEACTIVATION_THRESHOLD)
        {
            // We are done dispensing
            ALL_OUTPUTS_OFF();
            nextMode(TIMER_MODE_IR_WAIT);
            // Force battery check on IR wait entry
            gBattWaitCount = CHECK_BATT_COUNT;
        }
        // Have we reached the maximum 1.8s threshold?
        else if (gDispenseTime > MAX_DISPENSE_COUNT)
        {
            // Turn off the pump, but we aren't done "dispensing" until hand is removed.
            SET_PUMP(false);
            SET_WHITE_LED(false);
            // Just to make sure this counter is now pegged for next loop
            gDispenseTime = MAX_DISPENSE_COUNT;
            // Blink red LED if hand continues to be sensed for an extended period of time
            if (gSensorUnblockCount == IM_WAITING_BLINK_START_COUNT)
            {
                SET_RED_LED(true);
            }
            else if (gSensorUnblockCount == IM_WAITING_BLINK_LENGTH_COUNT)
            {
                SET_RED_LED(false);
            }
            else if (gSensorUnblockCount >= IM_WAITING_BLINK_DELAY_COUNT)
            {
                // Reset counter so it continues to blink
                gSensorUnblockCount = IM_WAITING_BLINK_START_COUNT - 1;
            }
            ++gSensorUnblockCount;
        }
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
              gPowerOnCount++;
              gPrankModeEnabled = (gPowerOnCount == PRANK_MODE_POWER_ON_COUNT);
          }
          ALL_OUTPUTS_OFF();
          SET_WHITE_LED(true);
          nextMode(TIMER_MODE_ENTER_RUN);
          ENABLE_OSCILLATOR_ON_EXIT(); // Essentially moving from LPM4 to LPM3
      }
      // Toggle to new state
      gPowerState = !gPowerState;
  }
  else if (P1IFG & P1_IPIN_IR_SENSE)
  {
      // Hand was sensed

      P1IFG &= ~P1_IPIN_IR_SENSE; // Clear interrupt flag
      gHandSensed = true;
  }
}

