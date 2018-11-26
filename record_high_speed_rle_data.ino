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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

//
// This function records data using Run Length Encoding (RLE).
// This is useful when the data does not change very often. If the
// value does not change from the previous value, it just increments
// a count. When the value changes again, it stores an extra sample
// with the count, along with the highest channel set to 1 (e.g.
// channel 7 for 8 channel mode, channel 3 for 4 channel mode). This
// means that the highest channel cannot be used for data.
//
void recordHighSpeedRLEData (sumpSetupVariableStruct &sv,
                             sumpDynamicVariableStruct &dynamic)
{
  // hard-code values since always 8 channels
  const byte samplesPerElement = 4;
  const byte samplesPerElementMinusOne = 3;
  const uint32_t sampleMask = 0x7F;
  const uint32_t sampleShift = 8;
  const uint32_t rleCountIndicator = 0x80;
  const uint32_t anyDataMask = 0x80808080;

// packed booleans can be written/read as fast as regular booleans
struct Packed_Bools {
  bool bufferHasWrapped : 1;
  bool doneRecording    : 1;
  bool rleInProgress    : 1;
  bool simpleTrigger    : 1;
  bool skipWait         : 1;
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

Packed_Bools_Union bools;
Packed_Union packed;
Packed_Union triggerArray[4];


////  bool bufferHasWrapped = false;
bools.b.bufferHasWrapped = false;
  int elementsToRecord = sv.samplesToRecord / sv.samplesPerElement;
  register uint32_t *inputPtr = (uint32_t *)sv.startOfBuffer;
  uint32_t *endOfBuffer = (uint32_t *)sv.endOfBuffer;
  uint32_t *startOfBuffer = (uint32_t *)sv.startOfBuffer;
  uint32_t *startPtr = (uint32_t *)sv.startOfBuffer;
////  byte samplesPerElement = sv.samplesPerElement;
////  byte samplesPerElementMinusOne = samplesPerElement - 1;
  
  // shift right 1 to mask off upper channel, which is used for RLE
////  uint32_t sampleMask = sv.sampleMask >> 1;
////  uint32_t anyDataMask = sv.anyDataMask;;

  // this is to set the highest channel for an RLE count
////  uint32_t rleCountIndicator = sv.rleCountIndicator;

////  uint32_t sampleShift = sv.sampleShift;
  int sampleValue = -1;
//  int previousSampleValue = -1;
  uint32_t previousFirstValue = 0;
////  bool rleInProgress = false;
bools.b.rleInProgress = false;
//  int triggerCount = samplesPerElementMinusOne;
  register int workingCount = samplesPerElementMinusOne + 1;
  register uint32_t workingValue = 0;

  bools.b.simpleTrigger = false;
  bools.b.skipWait = false;

  int currentTriggerLevel = 0;
////  uint32_t triggerMask = sv.triggerMask[0];
////  uint32_t triggerValue = sv.triggerValue[0];
////  uint32_t triggerDelay = sv.triggerDelay[0];
packed.p.triggerMask = sv.triggerMask[0];
packed.p.triggerValue = sv.triggerValue[0];
packed.p.triggerDelay = sv.triggerDelay[0];

  // state is not used except to make a switch context for ptr
  stateType state;
  // ptr points to a label inside the switch statement to speed it up,
  // since it doesn't have to calculate the jump table each time through.
  // Label names are the case names with '_Label' added
  register void *switch_ptr;

  // set up trigger array
  if (sv.lastTriggerLevel == 0)
  {
    currentTriggerLevel = 0;

    if (sv.triggerDelay[0] == 0)
    {
      bools.b.simpleTrigger = true;
    }
DEBUG_SERIAL (print("Packed int:"));
DEBUG_SERIAL (println(packed.integer, HEX));
  }
  else if (sv.lastTriggerLevel == 1)
  {
    currentTriggerLevel = 1;
    triggerArray[0].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
  }
  else if (sv.lastTriggerLevel == 2)
  {
    currentTriggerLevel = 2;
    triggerArray[1].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
    triggerArray[0].integer = sv.triggerMask[2] + (sv.triggerValue[2] << 8) + (sv.triggerDelay[2] << 16);
  }
  else if (sv.lastTriggerLevel == 3)
  {
    currentTriggerLevel = 3;
    triggerArray[2].integer = sv.triggerMask[1] + (sv.triggerValue[1] << 8) + (sv.triggerDelay[1] << 16);
    triggerArray[1].integer = sv.triggerMask[2] + (sv.triggerValue[2] << 8) + (sv.triggerDelay[2] << 16);
    triggerArray[0].integer = sv.triggerMask[3] + (sv.triggerValue[3] << 8) + (sv.triggerDelay[3] << 16);
  }

  // if using a trigger
  if (sv.triggerMask[0])
  {
    switch_ptr = &&Buffering_Label;

    // position to arm the trigger
    startPtr = inputPtr + sv.delaySizeInElements;
  }
  else
  {
    switch_ptr = &&Triggered_Second_Pass_Label;

    startPtr = endOfBuffer;
  }

  // 100% causes a problem with circular buffer - never stops
  if (startPtr >= endOfBuffer)
  {
    startPtr = endOfBuffer - 1;
  }

  maskInterrupts ();

  // read enough samples prior to arming to meet the pre-trigger request
  // (for speed, use while (1) and break instead of while (inputPtr != startPtr))
  while (1)
  {
/*
    if (bools.b.skipWait)
    {
      bools.b.skipWait = false;
      clearTimerFlag ();
    }
    else
*/
    {
////    waitForTimeout ();
waitForTimeout3 ();
    }

////Skip_Wait_Label:
////          digitalWriteFast (TIMING_PIN_1, LOW);
    // read a sample
//    sampleValue = PORT_DATA_INPUT_REGISTER & sampleMask;

if ((PORT_DATA_INPUT_REGISTER & sampleMask) != (uint32_t)sampleValue)
////if ((temp = (PORT_DATA_INPUT_REGISTER & sampleMask)) != sampleValue)
//    if (sampleValue != previousSampleValue)
    {
sampleValue = PORT_DATA_INPUT_REGISTER & sampleMask;
////sampleValue = temp;

        #ifdef TIMING_DISCRETES
toggleTimingPin1 ();
        #endif
//      bools.b.skipWait = true;

      // if previous rle count has not been written
      if (workingCount == 0)
      {
        #ifdef TIMING_DISCRETES_2
          digitalWriteFast (TIMING_PIN_3, HIGH);
        #endif

        *inputPtr = workingValue;
        ++inputPtr;

        // adjust for circular buffer wraparound at the end
        if (inputPtr >= endOfBuffer)
        {
          #ifdef TIMING_DISCRETES_2
            digitalWriteFast (TIMING_PIN_5, HIGH);
          #endif

          inputPtr = startOfBuffer;

          bools.b.bufferHasWrapped = true;

          // if any data is received from PC, then stop (assume it is a reset)
          if (usbInterruptPending ())
          {
            DEBUG_SERIAL(print(" Halt due to USB interrupt"));
            set_led_off ();
            SUMPreset();
            break;
          }

          #ifdef TIMING_DISCRETES_2
            digitalWriteFast (TIMING_PIN_5, LOW);
          #endif
        }

        // save new value (no need to shift since just saved)
        workingValue = sampleValue;
        workingCount = samplesPerElementMinusOne;

        #ifdef TIMING_DISCRETES_2
          digitalWriteFast (TIMING_PIN_3, LOW);
        #endif
      }
      else
      {
        // save new value
        workingValue = (workingValue << sampleShift) + sampleValue;
        --workingCount;
      }

//      previousSampleValue = sampleValue;
      bools.b.rleInProgress = false;
    }
    else  // same
    {
      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_2, HIGH);
      #endif

      if (bools.b.rleInProgress == false)
      {
        // save count for previous value
        workingValue = (workingValue << sampleShift) + rleCountIndicator + 1;
        --workingCount;

        bools.b.rleInProgress = true;
      }
      else
      {
        // number of RLE instances is stored in the working Value
        workingValue++;

        // if RLE count is at the maximum value
        if ((workingValue & sampleMask) == sampleMask)
        {
          #ifdef TIMING_DISCRETES_2
            digitalWriteFast (TIMING_PIN_4, HIGH);
          #endif

          // force current count to be written and new count started
          bools.b.rleInProgress = false;

          // not enough time to check if also going to write working value
          if (workingCount != 0)
          {
            // if any data is received from PC, then stop (assume it is a reset)
            if (usbInterruptPending ())
            {
              DEBUG_SERIAL(print(" Halt due to USB interrupt"));
              set_led_off ();
              SUMPreset();
              break;
            }
          }

          #ifdef TIMING_DISCRETES_2
            digitalWriteFast (TIMING_PIN_4, LOW);
digitalWriteFast (TIMING_PIN_4, HIGH);
digitalWriteFast (TIMING_PIN_4, LOW);
          #endif
        }
      }

      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_2, LOW);
      #endif
    }

    // save the working value when it is full
    if (workingCount == 0 && bools.b.rleInProgress == false)
    {
      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_3, HIGH);
      #endif

      *inputPtr = workingValue;
      ++inputPtr;

      // adjust for circular buffer wraparound at the end
      if (inputPtr >= endOfBuffer)
      {
        #ifdef TIMING_DISCRETES_2
          digitalWriteFast (TIMING_PIN_5, HIGH);
        #endif

        inputPtr = startOfBuffer;

        bools.b.bufferHasWrapped = true;

        // if any data is received from PC, then stop (assume it is a reset)
        if (usbInterruptPending ())
        {
          DEBUG_SERIAL(print(" Halt due to USB interrupt"));
          set_led_off ();
          SUMPreset();
          break;
        }

        #ifdef TIMING_DISCRETES_2
          digitalWriteFast (TIMING_PIN_5, LOW);
        #endif
      }

      workingCount = samplesPerElement;
      workingValue = 0;

      #ifdef TIMING_DISCRETES_2
        digitalWriteFast (TIMING_PIN_3, LOW);
      #endif
      
    }  // if workingCount == 0
    else if (workingCount == 1)
    {
      // just before overwriting old data, check if it has data in it.
      // This is to prevent having just RLE counts at the start of the
      // buffer because it wrapped arn overwrote the original first value.

      // if any data (i.e. not just RLE counts) in this value, save it
      if ((*inputPtr & anyDataMask) != anyDataMask)
      {
        previousFirstValue = *inputPtr;
      }
    }

    //
    // For speed, perform the switch statement using a pointer to
    // the current state and a goto. The switch statement has to
    // be in the code so the compiler sets it up properly, but
    // the goto uses the duplicate labels for each case.
    //
    goto *switch_ptr;

    switch (state) {
      case LookingForSimpleTrigger :
      LookingForSimpleTrigger_Label:
        // if trigger has occurred
        if ((sampleValue & packed.p.triggerMask) == packed.p.triggerValue)
        {
          // last location to save
          startPtr = inputPtr - sv.delaySizeInElements;

          // move to triggered state
//            state = Triggered_First_Pass;
switch_ptr = &&Triggered_First_Pass_Label;
          #ifdef TIMING_DISCRETES
            digitalWriteFast (TIMING_PIN_1, LOW);
          #endif

        }
        break;

      case LookingForTrigger :
      LookingForTrigger_Label:
        // if trigger has occurred
        if ((sampleValue & packed.p.triggerMask) == packed.p.triggerValue)
        {
          if (packed.p.triggerDelay > 0) {
//          state = TriggerDelay;
switch_ptr = &&TriggerDelay_Label;
          } else {
            // if last trigger level
            if (currentTriggerLevel == 0)
            {
              // last location to save
              startPtr = inputPtr - sv.delaySizeInElements;

              // move to triggered state
//            state = Triggered_First_Pass;
switch_ptr = &&Triggered_First_Pass_Label;
              #ifdef TIMING_DISCRETES
                digitalWriteFast (TIMING_PIN_1, LOW);
              #endif

            } else {

              #ifdef TIMING_DISCRETES
                digitalWriteFast (TIMING_PIN_1, LOW);
              #endif

              // advance to next trigger level
              --currentTriggerLevel;
              packed.integer = triggerArray[currentTriggerLevel].integer;

              #ifdef TIMING_DISCRETES
                digitalWriteFast (TIMING_PIN_1, HIGH);
              #endif
            }
          }
        }
        break;

        case TriggerDelay :
TriggerDelay_Label:
          --packed.p.triggerDelay;
          if (packed.p.triggerDelay == 0) {
            // if last trigger level
            if (currentTriggerLevel == 0) {
              // last location to save
              startPtr = inputPtr - sv.delaySizeInElements;

              // move to triggered state
//              state = Triggered_First_Pass;
switch_ptr = &&Triggered_First_Pass_Label;

            } else {
              --currentTriggerLevel;
              packed.integer = triggerArray[currentTriggerLevel].integer;
//              state = LookingForTrigger;
switch_ptr = &&LookingForTrigger_Label;
            }
          }
          break;

        case Triggered:
Triggered_Label:
          if (inputPtr == startPtr) {
            // done recording. Use a goto for speed so that
            // no 'if' needed to check for done in the main loop
            goto DoneRecording;
          }
          break;

        case Buffering:
Buffering_Label:
          // if enough data is buffered
          if (inputPtr >= startPtr)
          {
            // move to armed state
//            state = LookingForTrigger;
switch_ptr = &&LookingForTrigger_Label;
            set_led_on ();

            #ifdef TIMING_DISCRETES
              digitalWriteFast (TIMING_PIN_1, HIGH);
            #endif
          }
          break;

        case Triggered_Second_Pass:
Triggered_Second_Pass_Label:
          // adjust for circular buffer wraparound at the end.
          if (startPtr < startOfBuffer)
          {
            startPtr = startPtr + elementsToRecord;
          }

          // move to triggered state
//          state = Triggered;
switch_ptr = &&Triggered_Label;

          #ifdef TIMING_DISCRETES
            digitalWriteFast (TIMING_PIN_1, LOW);
          #endif
          break;

        case Triggered_First_Pass:
 Triggered_First_Pass_Label:
          // go as fast as possible to try to catch up from Triggered state
//          state = Triggered_Second_Pass;
switch_ptr = &&Triggered_Second_Pass_Label;
          set_led_off (); // TRIGGERED, turn off LED
          break;
      }
//    }  // if state == LookingForTrigger

  } // while (1)

  DoneRecording:

  // cleanup
  unmaskInterrupts ();

  #ifdef TIMING_DISCRETES
    digitalWriteFast (TIMING_PIN_0, LOW);
  #endif

  // save the first value in case it was overwritten due to buffer overflow
  // (i.e. first sample is an RLE count, but what was the value it is counting?)
  sv.firstRLEValue = previousFirstValue;

  // adjust trigger count
  dynamic.triggerSampleIndex = (startPtr + sv.delaySizeInElements - startOfBuffer) * samplesPerElement + samplesPerElementMinusOne;

  dynamic.bufferHasWrapped = bools.b.bufferHasWrapped;

  // adjust for circular buffer wraparound at the end.
  if (dynamic.triggerSampleIndex >= (uint32_t)sv.samplesToRecord)
  {
    dynamic.triggerSampleIndex = dynamic.triggerSampleIndex - sv.samplesToRecord;
  }

  if (inputPtr != startPtr)
  {
    int deltaElements = inputPtr - startOfBuffer;

    if (deltaElements < 0)
    {
      deltaElements += elementsToRecord;
    }

    dynamic.interruptedIndex = deltaElements * samplesPerElement;
  }
}

inline void toggleTimingPin0 () {
  *portToggleRegister (TIMING_PIN_0) = 1;
}
inline void toggleTimingPin1 () {
////  *portToggleRegister (TIMING_PIN_1) = 1;
*portToggleRegister (TIMING_PIN_3) = 1;
}

inline void waitForTimeout2 (void)
{
  #ifdef TIMING_DISCRETES
    toggleTimingPin0 ();
  #endif


  // for speed, to reduce jitter
  if (TIMER_FLAG_REGISTER)
  {
    clearTimerFlag ();  
  } else if (TIMER_FLAG_REGISTER) {
    clearTimerFlag ();  
  } else {
//    digitalWriteFast (TIMING_PIN_0, LOW);
//    while (!TIMER_FLAG_REGISTER);
//    digitalWriteFast (TIMING_PIN_0, HIGH);
//    clearTimerFlag ();  
//  }


   waitStart:
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (TIMER_FLAG_REGISTER) goto waitEnd;
   if (!TIMER_FLAG_REGISTER) goto waitStart;

   waitEnd:
   clearTimerFlag ();
}

  #ifdef TIMING_DISCRETES     
    toggleTimingPin0 ();
  #endif
}

inline void waitForTimeout3 (void)
{
  #ifdef TIMING_DISCRETES     
//    digitalWriteFast (TIMING_PIN_0, HIGH);
    toggleTimingPin0 ();
  #endif

  // WaitCount has to be less than cpu cycles in the shortest
  // loop (Do_Triggered?), so that it doesn't start too early
  const int WaitCount = 24 / (F_CPU / F_BUS);
  
  while (PIT_CVAL0 > WaitCount);

  #ifdef TIMING_DISCRETES     
//    digitalWriteFast (TIMING_PIN_0, LOW);
    toggleTimingPin0 ();
  #endif
}


