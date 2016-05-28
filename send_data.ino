/* Teensy Logic Analyzer
 * Copyright (c) 2016 LAtimes2
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
// These functions are used to send the data back to the PC after recording
//

// Definitions:
//    sample - one sample of data. May be 1 bit, 2 bits, 4 bits, or 8 bits,
//             as defined by sv.samplesPerElement, sampleMask, and sampleShift
//    element - collection of samples stored as one element. Currently 32 bit integer.
//              Oldest sample is in msb of element, newest sample is lsb
//    data - all the data that was recorded. An array of elements.
//    arrayIndex - index of elements in data (0-based)
//    elementIndex - index into an element. 0 is newest sample.
//    sampleIndex - index of samples in data. 0 is first address. 0 is not
//                  necessarily the oldest sample, since the buffer can wrap
//                  while waiting for a trigger.

byte getSample (sumpSetupVariableStruct setup, int sampleIndex)
{
  uint32_t sample;

  int arrayIndex = sampleIndex / setup.samplesPerElement;
  int elementIndex = setup.samplesPerElement - (sampleIndex % setup.samplesPerElement) - 1;

  sample = (setup.startOfBuffer[arrayIndex] >> (setup.sampleShift * elementIndex)) & setup.sampleMask;

  return sample;
}

void adjustTrigger (
  sumpSetupVariableStruct setup,
  sumpDynamicVariableStruct &dynamic)
{

#if HARDWARE_CONFIGURATION
  // look from 14 before to 16 after the current trigger index for actual trigger change
  // (hardware checks 32 bits at a time)
  const int startIndex = -14;
  const int endIndex = 16;
#else
  // assembly checks every 8 samples
  const int startIndex = -7;
  const int endIndex = 0;
#endif

  int currentStartIndex = startIndex;        // can get adjusted at each stage
  uint32_t triggerSampleIndex = 0xFFFFFFFF;  // set to large invalid value

  // for each trigger level
  for (int level = 0; level <= setup.lastTriggerLevel; ++level) {

    for (int index = currentStartIndex; index <= endIndex; ++index) {

      // since unsigned, check for wraparound to a very large number (i.e. negative)
      if (dynamic.triggerSampleIndex + index < 0xF0000000) {
        // if this is the trigger
        if ((getSample (setup, dynamic.triggerSampleIndex + index) & setup.triggerMask[level]) == setup.triggerValue[level]) {

          if (level == setup.lastTriggerLevel) {
            triggerSampleIndex = dynamic.triggerSampleIndex + index;

            // add trigger delay if not at start of search area (may have been true earlier)
            if (index != startIndex) {
              // add any delay after the trigger
              triggerSampleIndex += setup.triggerDelay[level];
            }

            // don't know why it needs this, but get it to align
            if (setup.numberOfChannels == 2) {
              dynamic.triggerSampleIndex -= 1;
            }
          } else {
            // not last level - continue with next stage
            currentStartIndex = index;

            // add trigger delay if not at start of search area (may have been true earlier)
            if (index != startIndex) {
              currentStartIndex += setup.triggerDelay[level];

              // if delay goes past end of search area, start searching at beginning again
              // (may have been a false trigger at this level, it was really before search area
              //  due to large delay)
              if (currentStartIndex > endIndex) {
                currentStartIndex = startIndex;
              }
            }
          }
          break;
        }
      }

    }

  }

  // do not go past end of search area
  if (triggerSampleIndex <= dynamic.triggerSampleIndex + endIndex) {
    dynamic.triggerSampleIndex = triggerSampleIndex;
  }
}

void sendData (
  sumpSetupVariableStruct sumpSetup,
  sumpDynamicVariableStruct dynamic)
{
  int firstSampleIndex;
  int lastSampleIndex;
  uint32_t triggerSampleIndex;
  byte sample;
  bool wrappedBuffer = false;

  // if using trigger
  if (sumpSetup.triggerMask[0])
  {
    adjustTrigger (sumpSetup, dynamic);
  }

  triggerSampleIndex = dynamic.triggerSampleIndex;

  // make sure it didn't adjust too far
  if (triggerSampleIndex < sumpSetup.delaySamples ||
      triggerSampleIndex > 0xFFFF0000)
  {
    triggerSampleIndex = sumpSetup.delaySamples;
  }

  // set unused channels to alternating 1's and 0's
  byte unusedValue = 0x55;

  // get first sampleIndex to send
  firstSampleIndex = triggerSampleIndex - sumpSetup.delaySamples;

  // get last sampleIndex to send
  lastSampleIndex = firstSampleIndex + sumpSetup.samplesToSend - 1;

  // if buffer wrapped around
  if (lastSampleIndex >= sumpSetup.samplesToRecord)
  {
    lastSampleIndex = lastSampleIndex - sumpSetup.samplesToRecord;
    wrappedBuffer = true;
  }


  // if samples were limited, send bogus data to indicate it is done.
  // Send these first since sent backwards in time
  for (int index = sumpSetup.samplesToSend; index < sumpSetup.samplesRequested; index = index + 2)
  {
    // send alternating 1's and 0's
    Serial.write(0x55);
    Serial.write(0xAA);
  }

  // send the data up to end of buffer
  int finalIndex;

  if (wrappedBuffer)
  {
    finalIndex = sumpSetup.samplesToRecord - 1;    
  }
  else
  {
    finalIndex = lastSampleIndex;
  }

  // data is send from last to first

  // if buffer wrapped, send the last part of the data
  if (wrappedBuffer)
  {
    for (int index = lastSampleIndex; index >= 0; --index)
    {
      sample = getSample (sumpSetup, index);
      sample = (unusedValue & ~sumpSetup.sampleMask) + (sample & sumpSetup.sampleMask);
      Serial.write(sample);

      unusedValue = ~unusedValue;   // toggle 1's and 0's
    }  
  }

  for (int index = finalIndex; index >= firstSampleIndex; --index)
  {
    sample = getSample (sumpSetup, index);
    sample = (unusedValue & ~sumpSetup.sampleMask) + (sample & sumpSetup.sampleMask);
    Serial.write(sample);

    unusedValue = ~unusedValue;   // toggle 1's and 0's
  }

}


