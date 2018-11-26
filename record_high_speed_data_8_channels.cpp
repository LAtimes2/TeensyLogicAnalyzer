/* Teensy Logic Analyzer
 * Copyright (c) 2018 LAtimes2
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIeDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "types.h"

// This file is a cpp file so that it can assign registers for just within this scope. 

// Teensy LC
#if defined(__MKL26Z64__)
  #define Teensy_LC 1
#endif

// LC doesn't have enough registers for high speed
#if not Teensy_LC

// use Port D for sampling
#define PORT_DATA_INPUT_REGISTER  GPIOD_PDIR

#define DEBUG_SERIAL(x) 0   // no debug output
//#define DEBUG_SERIAL(x) Serial2.x // debug output to Serial2

//#define TIMING_DISCRETES   // if uncommented, set pins 0 and 1 for timing

#define TIMING_PIN_0 15
#define TIMING_PIN_1 16
#define TIMING_PIN_2 17
#define TIMING_PIN_3 18
#define TIMING_PIN_4 19
#define TIMING_PIN_5 22

// these are from the main sketch
extern int getCurrentFBUS ();
extern void maskInterrupts (void);
extern void set_led_off ();
extern inline void set_led_on ();
extern void SUMPreset(void);
extern void unmaskInterrupts (void);
extern inline bool usbInterruptPending (void);

// forward declaration
inline void waitForTimeout (void);

// packed booleans can be written/read as fast as regular booleans
struct Packed_Bools {
  bool bufferHasWrapped : 1;
  bool doneRecording    : 1;
  bool simpleTrigger    : 1;
  bool rleInProgress    : 1;
  bool skipSomeWaits    : 1;
};

union Packed_Bools_Union {
  Packed_Bools b;
  int32_t integer;
};

// pack into a single int to fit in 1 register
struct Packed_Type {
  uint8_t triggerMask   : 8;
  uint8_t triggerValue  : 8;
  uint16_t triggerDelay : 16;
};

union Packed_Union {
  Packed_Type p;
  int32_t integer;
};

// save time by having structure pointer in a register
// and using an offset to get each element
struct Struct
{
  int currentTriggerLevel;
  uint32_t delaySizeInElements;
  int elementsToRecord;
  uint32_t *startOfBuffer;
  uint32_t *startPtr;
  uint32_t *triggerPtr;
};

Struct variable_struct;

// define registers for items that are accessed a lot
register Struct *struct_ptr asm("r5");
register uint32_t *endOfBuffer asm("r12");
register uint32_t *inputPtr asm("r9");
register uint32_t workingValue asm("r6");

volatile register uint8_t *port_data_register asm("r4");
volatile register uint32_t *timer_counter_register asm("r7");

register void (*stateFunctionPtr)() asm("r8");

register Packed_Bools_Union bools asm ("r11");
register Packed_Union packed asm ("r10");
Packed_Union triggerArray[4];

// forward declarations
void Do_LookingForSimpleTrigger ();
void Do_LookingForTrigger ();
void Do_Triggered_Second_Pass ();
void Do_TriggerFound ();


FASTRUN void Do_Buffering ()
{
  // if enough data is buffered
  if (inputPtr >= struct_ptr->startPtr)
  {
    // move to armed state
    if (bools.b.simpleTrigger)
    {
      stateFunctionPtr = &Do_LookingForSimpleTrigger;
    } else {
      stateFunctionPtr = &Do_LookingForTrigger;
    }

    set_led_on ();

    #ifdef TIMING_DISCRETES
      digitalWriteFast (TIMING_PIN_1, HIGH);
    #endif
  } else {
    waitForTimeout ();
  }
}

FASTRUN void Do_Triggered ()
{
  if (inputPtr == struct_ptr->startPtr)
  {
    bools.b.doneRecording = true;
  }

  waitForTimeout ();
}

FASTRUN void Do_Triggered_First_Pass ()
{
  // go as fast as possible to try to catch up from Triggered state
  stateFunctionPtr = &Do_Triggered_Second_Pass;

  set_led_off (); // TRIGGERED, turn off LED

  waitForTimeout ();
}

FASTRUN void Do_Triggered_Second_Pass ()
{
  // adjust for circular buffer wraparound at the end.
  if (struct_ptr->startPtr < struct_ptr->startOfBuffer)
  {
    struct_ptr->startPtr = struct_ptr->startPtr + struct_ptr->elementsToRecord;
  }

  // move to triggered state
  stateFunctionPtr = &Do_Triggered;

  #ifdef TIMING_DISCRETES
    digitalWriteFast (TIMING_PIN_2, HIGH);
  #endif  

  if (bools.b.skipSomeWaits)
  {
    return;
  }
  else
  {
    waitForTimeout ();
  }
}

FASTRUN void Do_TriggerDelay ()
{
  --packed.p.triggerDelay;
  if (packed.p.triggerDelay == 0)
  {
    struct_ptr->triggerPtr = inputPtr;

    // move to trigger found
    stateFunctionPtr = &Do_TriggerFound;
  }

  waitForTimeout ();
}

FASTRUN void Do_LookingForSimpleTrigger ()
{
  // if trigger has occurred
  if ((workingValue & packed.p.triggerMask) == packed.p.triggerValue)
  {
    // last location to save
    struct_ptr->startPtr = inputPtr - struct_ptr->delaySizeInElements;

    // move to triggered state
    stateFunctionPtr = &Do_Triggered_First_Pass;

    #ifdef TIMING_DISCRETES
      digitalWriteFast (TIMING_PIN_1, HIGH);
    #endif

    if (bools.b.skipSomeWaits)
    {
      return;
    }
    else
    {
      waitForTimeout ();
    }
  } else {
    waitForTimeout ();
  }
}

FASTRUN void Do_LookingForTrigger ()
{
  // if trigger has occurred
  if ((workingValue & packed.p.triggerMask) == packed.p.triggerValue)
  {
    if (packed.p.triggerDelay > 0) {
      stateFunctionPtr = &Do_TriggerDelay;
    } else {
      struct_ptr->triggerPtr = inputPtr;

      // move to triggered state
      stateFunctionPtr = &Do_TriggerFound;
    }

    if (bools.b.skipSomeWaits)
    {
      return;
    }
    else
    {
      waitForTimeout ();
    }
  } else {
    waitForTimeout ();
  }
}

FASTRUN void Do_TriggerFound ()
{
  if (struct_ptr->currentTriggerLevel == 0)
  {
    // last location to save
    struct_ptr->startPtr = struct_ptr->triggerPtr - struct_ptr->delaySizeInElements;

    // move to triggered state
    stateFunctionPtr = &Do_Triggered_First_Pass;

    #ifdef TIMING_DISCRETES
      digitalWriteFast (TIMING_PIN_1, HIGH);
    #endif


    if (bools.b.skipSomeWaits)
    {
      return;
    }
    else
    {
      waitForTimeout ();
    }
  } else {

    #ifdef TIMING_DISCRETES
      digitalWriteFast (TIMING_PIN_1, HIGH);
    #endif

    // advance to next trigger level
    --struct_ptr->currentTriggerLevel;
    packed.integer = triggerArray[struct_ptr->currentTriggerLevel].integer;

    // move to looking for next trigger level
    stateFunctionPtr = &Do_LookingForTrigger;

    #ifdef TIMING_DISCRETES
      digitalWriteFast (TIMING_PIN_1, LOW);
    #endif

    if (bools.b.skipSomeWaits)
    {
      return;
    }
    else
    {
      waitForTimeout ();
    }
  }
}


void recordHighSpeedData_8_Channels (
  sumpSetupVariableStruct &sv,
  sumpDynamicVariableStruct &dynamic)
{
  // backup the register values before using
  Packed_Bools_Union save_bools;
  save_bools.integer = bools.integer;
  Packed_Union save_packed;
  save_packed.integer = packed.integer;
  uint32_t *save_endOfBuffer = endOfBuffer;
  uint32_t *save_inputPtr = inputPtr;
  volatile uint8_t *save_port_data_register = port_data_register;
  Struct *save_struct_ptr = struct_ptr;
  void (*save_stateFunctionPtr)() = stateFunctionPtr;
  volatile uint32_t *save_timer_counter_register = timer_counter_register;
  uint32_t  save_workingValue = workingValue;

  port_data_register = (volatile uint8_t *)&PORT_DATA_INPUT_REGISTER;
  timer_counter_register = &PIT_CVAL0;
  
  struct_ptr = &variable_struct;
  bools.b.bufferHasWrapped = false;
  struct_ptr->elementsToRecord = sv.samplesToRecord / sv.samplesPerElement;
  inputPtr = sv.startOfBuffer;
  endOfBuffer = sv.endOfBuffer;
  struct_ptr->startOfBuffer = sv.startOfBuffer;
  struct_ptr->delaySizeInElements = sv.delaySizeInElements;
  struct_ptr->startPtr = sv.startOfBuffer;

  // hard-code values since always 8 channels
  const byte samplesPerElement = 4;
  const byte samplesPerElementMinusOne = 3;
  const uint32_t sampleMask = 0xFF;
  const uint32_t sampleShift = 8;

  workingValue = 0;

  struct_ptr->currentTriggerLevel = 0;

  packed.p.triggerValue = sv.triggerValue[0];
  packed.p.triggerMask += sv.triggerMask[0];
  packed.p.triggerDelay += sv.triggerDelay[0];

  // at the fastest speed, skip some WaitForTimeouts
  if (sv.cpuClockTicks <= 24)
  {
    bools.b.skipSomeWaits = true;
  }
  else
  {
    bools.b.skipSomeWaits = false;
  }
  
  bools.b.simpleTrigger = false;

  // set up trigger array
  if (sv.lastTriggerLevel == 0)
  {
    struct_ptr->currentTriggerLevel = 0;

    if (sv.triggerDelay[0] == 0)
    {
      bools.b.simpleTrigger = true;
    }
  }
  else if (sv.lastTriggerLevel == 1)
  {
    struct_ptr->currentTriggerLevel = 1;
    triggerArray[0].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
  }
  else if (sv.lastTriggerLevel == 2)
  {
    struct_ptr->currentTriggerLevel = 2;
    triggerArray[1].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
    triggerArray[0].integer = sv.triggerMask[2] + (sv.triggerValue[2] << 8) + (sv.triggerDelay[2] << 16);
  }
  else if (sv.lastTriggerLevel == 3)
  {
    struct_ptr->currentTriggerLevel = 3;
    triggerArray[2].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
    triggerArray[1].integer = sv.triggerMask[2] + (sv.triggerValue[2] << 8) + (sv.triggerDelay[2] << 16);
    triggerArray[0].integer = sv.triggerMask[3] + (sv.triggerValue[3] << 8) + (sv.triggerDelay[3] << 16);
  }

  // trigger delay is only checked 3 out of 4 passes, so adjust the end value accordingly
  for (int index = 0; index < sv.lastTriggerLevel; index++)
  {
    triggerArray[index].p.triggerDelay = (triggerArray[index].p.triggerDelay * 3) / 4;
  }
  packed.p.triggerDelay = (packed.p.triggerDelay * 3) / 4;

  // if using a trigger
  if (sv.triggerMask[0])
  {
    stateFunctionPtr = &Do_Buffering;

    // position to arm the trigger
    struct_ptr->startPtr = inputPtr + struct_ptr->delaySizeInElements;
  }
  else
  {
    stateFunctionPtr = &Do_Triggered_Second_Pass;

    struct_ptr->startPtr = endOfBuffer;
  }

  // 100% causes a problem with circular buffer - never stops
  if (struct_ptr->startPtr >= endOfBuffer)
  {
    struct_ptr->startPtr = endOfBuffer - 1;
  }

  bools.b.doneRecording = false;
  maskInterrupts ();

  // read the first value
  workingValue = *port_data_register;

  // read enough samples prior to arming to meet the pre-trigger request
  // (for speed, use while (1) and break instead of while (inputPtr != startPtr))
  while (1)
  {
    // workingCount = 3
    workingValue = (workingValue << sampleShift) + (*port_data_register & sampleMask);

    // perform current action for this state
    stateFunctionPtr ();

    // workingCount = 2
    workingValue = (workingValue << sampleShift) + (*port_data_register & sampleMask);

    stateFunctionPtr ();

    // workingCount = 1
    workingValue = (workingValue << sampleShift) + (*port_data_register & sampleMask);

    #ifdef TIMING_DISCRETES     
      digitalWriteFast (TIMING_PIN_1, HIGH);
    #endif

    *inputPtr = workingValue;
    ++inputPtr;

    // adjust for circular buffer wraparound at the end
    if (inputPtr >= endOfBuffer)
    {
      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_5, HIGH);
      #endif

      inputPtr = (uint32_t *)struct_ptr->startOfBuffer;

      bools.b.bufferHasWrapped = true;

      // if any data is received from PC, then stop (assume it is a reset)
      if (usbInterruptPending ())
      {
        DEBUG_SERIAL(print(" Halt due to USB interrupt"));
        set_led_off ();
        SUMPreset();
        goto DoneRecording;
        break;
      }

      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_5, LOW);
      #endif
    }
      
    waitForTimeout ();

    // delay to match other waits
    asm volatile ("nop\n\t");
    asm volatile ("nop\n\t");

    // first value after saving workingValue doesn't need to
    // shift previous value nor mask off extra bits. This saves
    // time that was used saving workingValue above.
    workingValue = *port_data_register;

    #ifdef TIMING_DISCRETES     
      digitalWriteFast (TIMING_PIN_1, LOW);
    #endif

    stateFunctionPtr ();

    if (bools.b.doneRecording)
    {
      goto DoneRecording;
    }
  } // while (1)

  DoneRecording:

  // cleanup
  unmaskInterrupts ();

  #ifdef TIMING_DISCRETES
    digitalWriteFast (TIMING_PIN_0, LOW);
  #endif

  // adjust trigger count
  dynamic.triggerSampleIndex = (struct_ptr->startPtr + sv.delaySizeInElements - struct_ptr->startOfBuffer) * samplesPerElement + samplesPerElementMinusOne;

  dynamic.bufferHasWrapped = bools.b.bufferHasWrapped;

  // adjust for circular buffer wraparound at the end.
  if (dynamic.triggerSampleIndex >= (uint32_t)sv.samplesToRecord)
  {
    dynamic.triggerSampleIndex = dynamic.triggerSampleIndex - sv.samplesToRecord;
  }

  // restore the register values
  bools.integer = save_bools.integer;
  packed.integer = save_packed.integer;
  endOfBuffer = save_endOfBuffer;
  inputPtr = save_inputPtr;
  port_data_register = save_port_data_register;
  struct_ptr = save_struct_ptr;
  stateFunctionPtr = save_stateFunctionPtr;
  timer_counter_register = save_timer_counter_register;
  workingValue = save_workingValue;
}

#define LED_PIN 13

inline void set_led_on () {
  digitalWriteFast (LED_PIN, HIGH);
}

inline void set_led_off () {
  digitalWriteFast (LED_PIN, LOW);
}

inline void toggleTimingPin0 () {
  *portToggleRegister (TIMING_PIN_0) = (uint32_t)stateFunctionPtr;
}

// returns true if a USB interrupt is pending (meaning data is available)
inline bool usbInterruptPending (void) {

  return (USB0_ISTAT & ~USB_ISTAT_SOFTOK);
}

inline void waitForTimeout (void)
{
  #ifdef TIMING_DISCRETES     
    toggleTimingPin0 ();
  #endif

  //if (*timer_counter_register < 14)

  // WaitCount has to be less than cpu cycles in the shortest
  // loop (Do_Triggered?), so that it doesn't start too early
#if F_CPU >= 144000000
  const int WaitCount = 14 / 2;
#else
  const int WaitCount = 14 / (F_CPU / F_BUS);
#endif
  
  asm volatile ("wait_loop_%=:\n\t"
                "ldr r2, [r7]\n\t"
                "cmp r2, %[WaitCount]\n\t"
                "bhi wait_loop_%=\n\t"

                :: [WaitCount] "i" (WaitCount)
                : "cc", "r2");

  #ifdef TIMING_DISCRETES     
    toggleTimingPin0 ();
  #endif
}

#endif  // if not Teensy_LC




