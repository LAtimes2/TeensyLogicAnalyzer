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

void recordLowSpeedData (sumpSetupVariableStruct &sv,
                         sumpDynamicVariableStruct &dynamic)
{
  bool bufferHasWrapped = false;
  int elementsToRecord = sv.samplesToRecord / sv.samplesPerElement;
  register uint32_t *inputPtr = (uint32_t *)sv.startOfBuffer;
  uint32_t *endOfBuffer = (uint32_t *)sv.endOfBuffer;
  uint32_t *startOfBuffer = (uint32_t *)sv.startOfBuffer;
  uint32_t *startPtr = (uint32_t *)sv.startOfBuffer;
  byte samplesPerElement = sv.samplesPerElement;
  byte samplesPerElementMinusOne = samplesPerElement - 1;
  register uint32_t sampleMask = sv.sampleMask;
  register uint32_t sampleShift = sv.sampleShift;
  int triggerCount = samplesPerElementMinusOne;
  register int workingCount = samplesPerElementMinusOne + 1;
  register uint32_t workingValue = 0;

  register stateType state = Buffering;
  int currentTriggerLevel = 0;
  register uint32_t triggerMask = sv.triggerMask[0];
  register uint32_t triggerValue = sv.triggerValue[0];
  uint32_t triggerDelay = sv.triggerDelay[0];

  // if using a trigger
  if (sv.triggerMask[0])
  {
    state = Buffering;

    // position to arm the trigger
    startPtr = inputPtr + sv.delaySizeInElements;
  }
  else
  {
    state = Triggered_Second_Pass;

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
    waitForTimeout ();

    // read a sample
    workingValue = (workingValue << sampleShift) + (PORT_DATA_INPUT_REGISTER & sampleMask);
    --workingCount;

    if (workingCount == 0)
    {
      *(inputPtr) = workingValue;
      ++inputPtr;

      // adjust for circular buffer wraparound at the end
      if (inputPtr >= endOfBuffer)
      {
        #ifdef TIMING_DISCRETES
          digitalWriteFast (TIMING_PIN_5, HIGH);
        #endif

        inputPtr = startOfBuffer;

        bufferHasWrapped = true;

        // if any data is received from PC, then stop (assume it is a reset)
        if (usbInterruptPending ())
        {
          DEBUG_SERIAL(print(" Halt due to USB interrupt"));
          set_led_off ();
          SUMPreset();
          break;
        }

        #ifdef TIMING_DISCRETES
          digitalWriteFast (TIMING_PIN_5, HIGH);
        #endif
      }
      
      workingCount = samplesPerElement;
    }  // if workingCount == 0

    // this state cannot afford enough time for the switch statement
    if (state == LookingForTrigger)
    {
      // if trigger has occurred
      if ((workingValue & triggerMask) == triggerValue) {

        if (triggerDelay > 0) {
          state = TriggerDelay;
        } else {
          // if last trigger level
          if (currentTriggerLevel >= sv.lastTriggerLevel) {

            triggerCount = workingCount;
      
            // last location to save
            startPtr = inputPtr - sv.delaySizeInElements;

            // move to triggered state
            state = Triggered_First_Pass;

            #ifdef TIMING_DISCRETES
              digitalWriteFast (TIMING_PIN_1, LOW);
            #endif

          } else {

            #ifdef TIMING_DISCRETES
              digitalWriteFast (TIMING_PIN_1, LOW);
            #endif

            // advance to next trigger level
            ++currentTriggerLevel;
            triggerMask = sv.triggerMask[currentTriggerLevel];
            triggerValue = sv.triggerValue[currentTriggerLevel];

            #ifdef TIMING_DISCRETES
              digitalWriteFast (TIMING_PIN_1, HIGH);
            #endif
          }
        }
      }
    } else {

      switch (state) {
        case LookingForTrigger :
          // already done above
          break;

        case LookingForSimpleTrigger :
          // not used at low speeds
          break;

        case TriggerDelay :
          --triggerDelay;
          if (triggerDelay == 0) {
            // if last trigger level
            if (currentTriggerLevel >= sv.lastTriggerLevel) {
              triggerCount = workingCount;
      
              // last location to save
              startPtr = inputPtr - sv.delaySizeInElements;

              // move to triggered state
              state = Triggered_First_Pass;

            } else {
              ++currentTriggerLevel;
              triggerMask = sv.triggerMask[currentTriggerLevel];
              triggerValue = sv.triggerValue[currentTriggerLevel];
              triggerDelay = sv.triggerDelay[currentTriggerLevel];
              state = LookingForTrigger;
            }
          }
          break;

        case Triggered:
          if (inputPtr == startPtr) {
            // done recording
            goto DoneRecording;
          }
          break;

        case Buffering:
          // if enough data is buffered
          if (inputPtr >= startPtr)
          {
            // move to armed state
            state = LookingForTrigger;
            set_led_on ();

            #ifdef TIMING_DISCRETES
              digitalWriteFast (TIMING_PIN_1, HIGH);
            #endif
          }
          break;

        case Triggered_Second_Pass:
          // adjust for circular buffer wraparound at the end.
          if (startPtr < startOfBuffer)
          {
            startPtr = startPtr + elementsToRecord;
          }

          // move to triggered state
          state = Triggered;

          #ifdef TIMING_DISCRETES
            digitalWriteFast (TIMING_PIN_1, LOW);
          #endif
          break;

        case Triggered_First_Pass:
          // go as fast as possible to try to catch up from Triggered state
          state = Triggered_Second_Pass;
          set_led_off (); // TRIGGERED, turn off LED
          break;
      }
    }  // if state == LookingForTrigger

  } // while (1)

  DoneRecording:

  // cleanup
  unmaskInterrupts ();

  #ifdef TIMING_DISCRETES
    digitalWriteFast (TIMING_PIN_0, LOW);
  #endif

  // adjust trigger count
  dynamic.triggerSampleIndex = (startPtr + sv.delaySizeInElements - startOfBuffer) * samplesPerElement + samplesPerElementMinusOne - triggerCount;

  dynamic.bufferHasWrapped = bufferHasWrapped;

  // adjust for circular buffer wraparound at the end.
  if (dynamic.triggerSampleIndex >= (uint32_t)sv.samplesToRecord)
  {
    dynamic.triggerSampleIndex = dynamic.triggerSampleIndex - sv.samplesToRecord;
  }
}





