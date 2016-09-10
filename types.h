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

enum stateType {
  Buffering,
  LookingForTrigger,
  TriggerDelay,
  Triggered_First_Pass,
  Triggered_Second_Pass,
  Triggered
};

enum TriggerType {
  Channel0High,
  Channel0Low,
  Channel1High,
  Channel1Low,
  BothChannelsHigh,
  BothChannelsLow,
  HighLow,
  LowHigh
};
  
// data to set up recording
struct sumpSetupVariableStruct {
  uint32_t busClockDivisor;
  uint32_t cpuClockTicks;
  uint32_t clockFrequency;
  uint32_t delaySamples = 0;
  uint32_t delaySizeInElements = 0;
  uint32_t numberOfChannels = 8;
  uint32_t sampleMask = 0xFF;
  uint32_t sampleShift = 8;
  uint32_t samplesPerElement = 4;
  uint32_t samplesRequested = 0;
  uint32_t samplesToRecord = 0;
  uint32_t samplesToSend;
  int lastTriggerLevel;
  byte triggerMask[4];
  byte triggerValue[4];
  uint16_t triggerDelay[4];
  uint32_t *startOfBuffer;
  uint32_t *endOfBuffer;
};

// data that changes while recording
struct sumpDynamicVariableStruct {
  uint32_t triggerSampleIndex;
};

