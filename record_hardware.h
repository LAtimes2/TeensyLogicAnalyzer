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
// This file is designed to be included multiple times with different
// preprocessor directives. This will provide maximum speed at the
// expense of extra code memory, which is currently not an issue.
//

bool checkTrigger (
  TriggerType triggerType,
  uint16_t sampleChan0,
  uint16_t sampleChan1);

// various function names
#if MULTIPLE_CHANNELS

  #if USE_TRIGGER
void recordSPIData_MultiChannel_Trigger (sumpSetupVariableStruct &sv,
                                         sumpDynamicVariableStruct &dynamic)
  #elif USE_PRE_TRIGGER
void recordSPIData_MultiChannel_Pretrigger (sumpSetupVariableStruct &sv,
                                            sumpDynamicVariableStruct &dynamic)
  #else
void recordSPIData_MultiChannel (sumpSetupVariableStruct &sv,
                                 sumpDynamicVariableStruct &dynamic)
  #endif

#else

  #if USE_TRIGGER
void recordSPIData_SingleChannel_Trigger (sumpSetupVariableStruct &sv,
                                          sumpDynamicVariableStruct &dynamic)
  #elif USE_PRE_TRIGGER
void recordSPIData_SingleChannel_Pretrigger (sumpSetupVariableStruct &sv,
                                             sumpDynamicVariableStruct &dynamic)
  #else
void recordSPIData_SingleChannel (sumpSetupVariableStruct &sv,
                                  sumpDynamicVariableStruct &dynamic)
  #endif

#endif

{

#define SPI_SR_TXCTR_MASK 0x0000F000
#define SPI_SR_RXCTR_MASK 0x000000F0

  uint32_t *startOfBuffer = (uint32_t *)sv.startOfBuffer;
  register uint32_t *inputPtr = (uint32_t *)sv.startOfBuffer;
  register uint32_t *endOfBuffer = (uint32_t *)sv.endOfBuffer;
  register uint32_t *startPtr = startOfBuffer;

  register uint32_t sampleChan0;

  bool bufferHasWrapped = false;
  byte triggerMask = sv.triggerMask[0];
  byte triggerValue = sv.triggerValue[0];

#if MULTIPLE_CHANNELS
  register uint32_t sampleChan1;

  // ignore unused channels
  triggerMask &= 0x03;
  triggerValue &= 0x03;
#else
  // ignore unused channels
  triggerMask &= 0x01;
  triggerValue &= 0x01;
#endif

#if USE_TRIGGER || USE_PRE_TRIGGER

  TriggerType triggerType = Channel0Low;

#if USE_TRIGGER
  int elementsToRecord = sv.samplesToRecord / sv.samplesPerElement;
  byte samplesPerElementMinusOne = sv.samplesPerElement - 1;
  register stateType state = Buffering;
  bool triggered = false;
#endif

  // if using a trigger
  if (triggerMask)
  {
#if USE_TRIGGER
    state = Buffering;

    // position to arm the trigger (add 6 to clear FIFO at start)
    startPtr = inputPtr + sv.delaySizeInElements + 6;
#else  // pre-trigger

    // set delay to 0
    sv.delaySamples = 0;
    sv.delaySizeInElements = 0;

    // position to stop recording
    // (assumes multiple of 2 ints')
    startPtr = endOfBuffer;
#endif  

    if (triggerMask == 1 && triggerValue == 0) triggerType = Channel0Low;
    if (triggerMask == 1 && triggerValue == 1) triggerType = Channel0High;
    if (triggerMask == 2 && triggerValue == 0) triggerType = Channel1Low;
    if (triggerMask == 2 && triggerValue == 2) triggerType = Channel1High;
    if (triggerMask == 3 && triggerValue == 0) triggerType = BothChannelsLow;
    if (triggerMask == 3 && triggerValue == 1) triggerType = HighLow;
    if (triggerMask == 3 && triggerValue == 2) triggerType = LowHigh;
    if (triggerMask == 3 && triggerValue == 3) triggerType = BothChannelsHigh;  
  }
  else
  {
#if USE_TRIGGER
    state = Triggered_First_Pass;
#endif

    startPtr = endOfBuffer;
  }

#else  // no trigger
  startPtr = endOfBuffer;
#endif

  // 100% causes a problem with circular buffer - never stops
  if (startPtr >= endOfBuffer)
  {
    startPtr = endOfBuffer - 2;
  }

  spi1Initialize ();
  spi1Setup (sv.clockFrequency);

  #if MULTIPLE_CHANNELS
  {
    spi0Initialize ();
    spi0Setup (sv.clockFrequency);
  }
  #endif

  maskInterrupts ();

#if USE_PRE_TRIGGER
  //////////////////////
  //
  //   pre-trigger
  //
  //////////////////////
  uint16_t sample_chan0;
  uint16_t sample_chan1;
  bool done = false;

  // if using a trigger
  if (sv.triggerMask[0])
  {
    while (!done)
    {
      sample_chan0 = (digitalReadFast (CHAN0) ? 0xFFFF : 0);
      sample_chan1 = (digitalReadFast (CHAN1) ? 0xFFFF : 0);

      done = checkTrigger (
        triggerType,
        sample_chan0,
        sample_chan1);

      // if any data is received from PC, then stop (assume it is a reset)
      if (usbInterruptPending ())
      {
        DEBUG_SERIAL(print(" Halt due to USB interrupt"));
        set_led_off ();
        SUMPreset();
        unmaskInterrupts ();
        return;
      }
    }
  }
#endif
  
  #if MULTIPLE_CHANNELS
    startSPIClock (true, sv.cpuClockTicks);
  #else
    startSPIClock (false, sv.cpuClockTicks);
  #endif

  //////////////////////
  //
  //   main loop
  //
  //////////////////////
  while (1)
  {

#if Teensy_LC

    // if data is ready to read
    if (SPI1_S & SPI_S_SPRF) {
      sampleChan0 = SPI1_DL | (SPI1_DH << 8);

      spi1StartTransfer ();

      #if MULTIPLE_CHANNELS
      {
        // clear status
        SPI0_S;

        sampleChan1 = SPI0_DL | (SPI0_DH << 8);

        spi0StartTransfer ();
      }
      #endif

      // wait until data is ready to read
      while (!(SPI1_S & SPI_S_SPRF));

      *(inputPtr) = sampleChan0 = (sampleChan0 << 16) + SPI1_DL + (SPI1_DH << 8);
      ++inputPtr;

      spi1StartTransfer ();

      #if MULTIPLE_CHANNELS
      {
        // clear status
        SPI0_S;

        *(inputPtr) = sampleChan1 = (sampleChan1 << 16) + SPI0_DL + (SPI0_DH << 8);
        ++inputPtr;

        spi0StartTransfer ();
      }
      #endif
 
#elif Teensy_3_5 || Teensy_3_6

    // if data is ready to read
    if (SPI1_SR & SPI_SR_RXCTR_MASK)
    {
      sampleChan0 = SPI1_POPR;

      // start next transfer
      SPI1_PUSHR = SPI_PUSHR_CONT;

      #if MULTIPLE_CHANNELS
      {
        sampleChan1 = SPI0_POPR;

        SPI0_PUSHR = SPI_PUSHR_CONT;
      }
      #endif

      // wait until data is ready to read
      while (!(SPI1_SR & SPI_SR_RXCTR_MASK));

      *(inputPtr) = sampleChan0 = (sampleChan0 << 16) + SPI1_POPR;
      ++inputPtr;

      // start next transfer
      SPI1_PUSHR = SPI_PUSHR_CONT;

      #if MULTIPLE_CHANNELS
      {
        *(inputPtr) = sampleChan1 = (sampleChan1 << 16) + SPI0_POPR;
        ++inputPtr;

        // start next transfer
        SPI0_PUSHR = SPI_PUSHR_CONT;
      }
      #endif
 
#else

    // if at least 2 data values are ready to read
    // (for speed, want constant mask of 0E0, so 'and' 0F0 and 1E0)
    if (SPI1_SR & SPI_SR_RXCTR_MASK & (SPI_SR_RXCTR_MASK << 1))
    {
      *(inputPtr) = sampleChan0 = (SPI1_POPR << 16) + SPI1_POPR;
      ++inputPtr;

      // start next transfer
      SPI1_PUSHR = SPI_PUSHR_CONT;
      SPI1_PUSHR = SPI_PUSHR_CONT;

      #if MULTIPLE_CHANNELS
      {
        *(inputPtr) = sampleChan1 = (SPI0_POPR << 16) + SPI0_POPR;
        ++inputPtr;

        SPI0_PUSHR = SPI_PUSHR_CONT;
        SPI0_PUSHR = SPI_PUSHR_CONT;
      }
      #endif

#endif

#if !USE_TRIGGER
      if (inputPtr == startPtr) {
        // done recording
        break;
      }
#else
      // adjust for circular buffer wraparound at the end
      if (inputPtr >= endOfBuffer)
      {
        inputPtr = sv.startOfBuffer;

        bufferHasWrapped = true;

        // if any data is received from PC, then stop (assume it is a reset)
        if (usbInterruptPending ())
        {
          DEBUG_SERIAL(print(" Halt due to USB interrupt"));
          set_led_off ();
          SUMPreset();
          break;
        }
      }

      switch (state)
      {
        case LookingForTrigger :
          // if trigger has occurred
          switch (triggerType)
          {
            case Channel0High:
              triggered = (sampleChan0 != 0);
              break;
            case Channel0Low:
              triggered = sampleChan0 != 0xFFFFFFFF;
              break;
#if MULTIPLE_CHANNELS
            case Channel1High:
              triggered = sampleChan1 != 0;
              break;
            case Channel1Low:
              triggered = sampleChan1 != 0xFFFFFFFF;
              break;
            case BothChannelsHigh:
              triggered = (sampleChan0 & sampleChan1) != 0;
              break;
            case BothChannelsLow:
              triggered = (sampleChan0 | sampleChan1) != 0xFFFFFFFF;
              break;
            case HighLow:
              triggered = (sampleChan0 & ~sampleChan1) != 0;
              break;
            case LowHigh:
              triggered = (sampleChan0 | ~sampleChan1) != 0xFFFFFFFF;
              break;
#endif
            default:
              break;
          }
          if (triggered)
          {
            // subtract 2 since inputPtr has already been incremented
            startPtr = inputPtr - 2 - sv.delaySizeInElements;

            // move to triggered state
            state = Triggered_First_Pass;
          }
          break;

        case TriggerDelay :
          break;

        case Triggered :
          if (inputPtr == startPtr)
          {
            // done recording
            goto DoneRecording;
          }
          break;

        case Buffering :
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

        case Triggered_First_Pass :
          // go as fast as possible to try to catch up from Triggered state
          state = Triggered_Second_Pass;
          break;

        case Triggered_Second_Pass :
          // adjust for circular buffer wraparound at the end.
          if (startPtr < startOfBuffer)
          {
            startPtr = startPtr + elementsToRecord;
          }

          // move to triggered state
          state = Triggered;
          set_led_off (); // TRIGGERED, turn off LED

          #ifdef TIMING_DISCRETES
            digitalWriteFast (TIMING_PIN_1, LOW);
          #endif
          break;
      } // end switch
#endif

    }  // if data is ready to be read

  } // while (1)

#if USE_TRIGGER
DoneRecording:
#endif

  unmaskInterrupts ();

#if USE_TRIGGER
  // set trigger count
  dynamic.triggerSampleIndex = (startPtr + sv.delaySizeInElements - startOfBuffer) * sv.samplesPerElement + samplesPerElementMinusOne;

  // adjust for circular buffer wraparound at the end.
  if (dynamic.triggerSampleIndex >= (uint32_t)sv.samplesToRecord)
  {
    dynamic.triggerSampleIndex = dynamic.triggerSampleIndex - sv.samplesToRecord;
  }

#else
  dynamic.triggerSampleIndex = sv.samplesPerElement;
#endif

  dynamic.bufferHasWrapped = bufferHasWrapped;

  // turn off SPI module
  #if MULTIPLE_CHANNELS
    spiDisable (true);
  #else
    spiDisable (false);
  #endif
}

// only need to include this routine once
#ifndef CHECKTRIGGER_DEFINED
#define CHECKTRIGGER_DEFINED

inline bool checkTrigger (
  TriggerType triggerType,
  uint16_t sampleChan0,
  uint16_t sampleChan1)
{
  bool triggered = false;

  // if trigger has occurred
  switch (triggerType)
  {
    case Channel0High:
      triggered = sampleChan0 != 0;
      break;
    case Channel0Low:
      triggered = sampleChan0 != 0xFFFF;
      break;
    case Channel1High:
      triggered = sampleChan1 != 0;
      break;
    case Channel1Low:
      triggered = sampleChan1 != 0xFFFF;
      break;
    case BothChannelsHigh:
      triggered = (sampleChan0 & sampleChan1) != 0;
      break;
    case BothChannelsLow:
      triggered = (sampleChan0 | sampleChan1) != 0xFFFF;
      break;
    case HighLow:
      triggered = (sampleChan0 & ~sampleChan1) != 0;
      break;
    case LowHigh:
      triggered = (sampleChan0 | ~sampleChan1) != 0xFFFF;
      break;
    default:
      break;
  }

  return triggered;
}

#endif
