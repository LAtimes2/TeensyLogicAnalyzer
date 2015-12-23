/* Teensy Logic Analyzer
 * Copyright (c) 2015 LAtimes2
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
//    arrayIndex - index into logicData (0-based)
//    elementIndex - index into an element. 0 is newest sample.
//    sampleIndex - index of samples in logicData. 0 is first address. 0 is not
//                  necessarily the oldest sample, since the buffer can wrap
//                  while waiting for a trigger.

byte getSample (struct sumpSetupVariableStruct setup, int sampleIndex)
{
  uint32_t sample;

  int arrayIndex = sampleIndex / setup.samplesPerElement;
  int elementIndex = setup.samplesPerElement - (sampleIndex % setup.samplesPerElement) - 1;

  sample = (setup.startOfBuffer[arrayIndex] >> (setup.sampleShift * elementIndex)) & setup.sampleMask;

DEBUG_SERIAL(print((int)setup.samplesPerElement));
DEBUG_SERIAL(print(", s: "));
DEBUG_SERIAL(print((int)sampleIndex, HEX));
DEBUG_SERIAL(print(", a: "));
DEBUG_SERIAL(print((int)arrayIndex, HEX));
DEBUG_SERIAL(print(", e: "));
DEBUG_SERIAL(print((int)elementIndex, HEX));
DEBUG_SERIAL(print(", sample: "));
DEBUG_SERIAL(print(sample, HEX));
DEBUG_SERIAL(println(""));
  return sample;
}

void sendData (
  struct sumpSetupVariableStruct sumpSetup,
  struct sumpDynamicVariableStruct dynamic)
{
  int firstSampleIndex;
  int lastSampleIndex;
  int triggerSampleIndex = dynamic.triggerSampleIndex;
  byte sample;
  bool wrappedBuffer = false;

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

DEBUG_SERIAL(print("triggerSampleIndex : "));
DEBUG_SERIAL(print((int)triggerSampleIndex, HEX));
DEBUG_SERIAL(print(", firstSampleIndex : "));
DEBUG_SERIAL(print((int)firstSampleIndex, HEX));
DEBUG_SERIAL(print(", lastSampleIndex : "));
DEBUG_SERIAL(print((int)lastSampleIndex, HEX));
DEBUG_SERIAL(println(""));

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
//DEBUG_SERIAL(print(", finalIndex : "));
//DEBUG_SERIAL(print((int)finalIndex, HEX));
//DEBUG_SERIAL(println(""));
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


