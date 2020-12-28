#include <msp430.h> 
#include <stdint.h>
#include <stdbool.h>

inline void outputIrStop();
inline void setupLpm3Interrupt(uint16_t counterMax, int8_t mode);
inline bool isBattLow();

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
#define SET_WHITE_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_WHITE_LED, on)
#define SET_RED_LED(on) SET_OUTPUT(P1OUT, P1_OPIN_RED_LED, on)
#define SET_IR_ACTIVE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_ACTIVE, on)
#define SET_PUMP(on) SET_OUTPUT(P1OUT, P1_OPIN_PUMP, on)
#define SET_IR_PULSE(on) SET_OUTPUT(P1OUT, P1_OPIN_IR_PULSE, on)
// All control IO are on port 1
#define ALL_OUTPUTS_OFF() P1OUT &= ~ALL_P1_OUTPUTS_MASK

// External crystal frequency
#define ACLK_FREQ 32768
// Go to LPM3 on exit of interrupt
#define LPM3_ON_EXIT() __bis_SR_register_on_exit(LPM3_bits)
// Go to LPM4 on exit of interrupt
#define LPM4_ON_EXIT() __bis_SR_register_on_exit(LPM4_bits)
// To be called within context of an ISR (float math only done by precompiler)
#define NEXT_MODE_ON_EXIT_MS(ms, mode) setupLpm3Interrupt((ms / 1000.0) * ACLK_FREQ - 1, mode); LPM3_ON_EXIT()
#define NEXT_MODE_ON_EXIT_COUNTS(counts, mode) setupLpm3Interrupt(counts, mode); LPM3_ON_EXIT()

// Length of time to turn on the ON LED and OFF LED
#define ON_LED_TIME_MS 1000
#define OFF_LED_TIME_MS 1000

// Frequency and counter value for IR sensor
#define IR_FREQUNCY 2340
#define IR_CHECK_COUNTER ACLK_FREQ / IR_FREQUNCY / 2 - 1

// Definitions of how many pulses we make for each IR check
#define IR_SENSE_PULSES 5
#define IR_SENSE_TRANSITIONS (IR_SENSE_PULSES * 2)
// Number of pulses within IR_SENSE_PULSES that must have sense before activation
#define IR_ACTIVATION_THRESHOLD 3
#define MAX_IR_SENSE_TIME_MS (IR_SENSE_PULSES * 1000 / IR_FREQUNCY)
#if IR_ACTIVATION_THRESHOLD >= IR_SENSE_PULSES
#error "activation value must be less than IR_ACTIVATION_THRESHOLD"
#endif

// Definitions for dispense
// The number of CONSECUTIVE pulses while dispensing must not have sense before deactivation
#define IR_DEACTIVATION_THRESHOLD 20
#define MIN_DISPENSE_TIME_S 0.9
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

// State machine modes
#define TIMER_MODE_NONE         0
#define TIMER_MODE_IR_WAIT      1
#define TIMER_MODE_IR_CHECK     2
#define TIMER_MODE_DISPENSE     3
#define TIMER_MODE_ENTER_RUN    4
#define TIMER_MODE_EXIT_RUN     5

// Current state machine mode
int8_t gTimerMode = TIMER_MODE_NONE;

// Current power state (true=ON; false=OFF)
bool gPowerState = false;
// Current IR state count
int8_t gIrCount = 0;
// Number of pulses where hand is sensed
int8_t gSenseCount = 0;
// Set by hand sense input interrupt
bool gHandSensed = false;
// Current dispense time counter (used in TIMER_MODE_DISPENSE)
int16_t gDispenseTime = 0;
// Wait counter for battery detection activation
int16_t gBattWaitCount = 0;
// Counter used to keep track of how long we have been waiting
int16_t gSensorUnblockCount = 0;
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

    // Finally, enable interrupts and enter LPM4 (initialized as powered off)
    __enable_interrupt();
    while(1)
    {
        // We should only reach here 0 or 1 times
        // This whole program is interrupt driven, so the chip should be in LPM3 or LPM4,
        // waiting from the next interrupt. LPM3 keeps the timer active while LPM4 turns that off.
        // This means that the timer interrupt will only be active in LPM3.
        LPM4;
        // Should not reach here, but just in case...
        ALL_OUTPUTS_OFF();
    }

    // Unreachable
    return 0;
}

inline void setupLpm3Interrupt(uint16_t counterMax, int8_t mode)
{
    CCTL0 = 0; // Make sure interrupt is disabled
    gTimerMode = mode;
    CCR0 = counterMax;
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

//Timer ISR
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer_A0(void)
{
    switch(gTimerMode)
    {
    case TIMER_MODE_IR_WAIT:
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
        // This will allow the sense line to go high before first entry
        NEXT_MODE_ON_EXIT_COUNTS(IR_CHECK_COUNTER, TIMER_MODE_IR_CHECK);
        break;
    case TIMER_MODE_IR_CHECK:
        if (gIrCount < IR_SENSE_TRANSITIONS)
        {
            ++gIrCount;
            if (GET_IR_PULSE())
            {
                // Currently high; check if hand is sensed then go low
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
                        gSenseCount = 0;
                        gSensorUnblockCount = 0;
                        SET_PUMP(true);
                        NEXT_MODE_ON_EXIT_COUNTS(IR_CHECK_COUNTER, TIMER_MODE_DISPENSE);
                    }
                }
                SET_IR_PULSE(false);
            }
            else
            {
                // Go high then wait for next interrupt
                SET_IR_PULSE(true);
            }
        }
        else
        {
            // All pulses were made without sensing hand; stop and go to wait state
            outputIrStop();
            NEXT_MODE_ON_EXIT_MS(IR_WAIT_PERIOD_MS, TIMER_MODE_IR_WAIT);
        }
        break;
    case TIMER_MODE_DISPENSE:
        ++gDispenseTime;
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
                        gSenseCount = 0x7F; // Max int8
                    }
                }
                else
                {
                    // Reset sense count to enforce that we get consecutive samples
                    gSenseCount = 0;
                    gHandSensed = false;
                }
                SET_IR_PULSE(false);
            }
            else
            {
                SET_IR_PULSE(true);
            }
        }
        // Have we reached the minimum 0.9s threshold?
        if (gDispenseTime > MIN_DISPENSE_COUNT)
        {
            // Is the hand now removed from the sensor?
            if (gSenseCount >= IR_DEACTIVATION_THRESHOLD)
            {
                // We are done dispensing
                ALL_OUTPUTS_OFF();
                NEXT_MODE_ON_EXIT_MS(IR_WAIT_PERIOD_MS, TIMER_MODE_IR_WAIT);
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
        break;
    case TIMER_MODE_ENTER_RUN:
        // Enter run mode
        ALL_OUTPUTS_OFF();
        NEXT_MODE_ON_EXIT_MS(IR_WAIT_PERIOD_MS, TIMER_MODE_IR_WAIT);
        break;
    case TIMER_MODE_EXIT_RUN: // Fall through
    case TIMER_MODE_NONE: // Fall through
    default:
        // Disable this interrupt, power off all outputs, and go to LPM4
        CCTL0 = 0;
        ALL_OUTPUTS_OFF();
        SET_RED_LED(false);
        __delay_cycles(5); // I think this helps to make sure outputs are set before going into LPM4
        LPM4_ON_EXIT(); // Go to LPM4 on exit of this interrupt (LPM4_EXIT only on button press)
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
          ALL_OUTPUTS_OFF();
          SET_RED_LED(true);
          NEXT_MODE_ON_EXIT_MS(OFF_LED_TIME_MS, TIMER_MODE_EXIT_RUN);
      }
      else
      {
          // Go from off to on
          LPM4_EXIT; // Exit LPM4
          ALL_OUTPUTS_OFF();
          SET_WHITE_LED(true);
          NEXT_MODE_ON_EXIT_MS(ON_LED_TIME_MS, TIMER_MODE_ENTER_RUN);
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

