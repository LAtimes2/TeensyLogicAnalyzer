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

// see record_asm_t4 for Teensy 4
#if not Teensy_4_0

#if not Teensy_LC
void recordDataAsm5Clocks (sumpSetupVariableStruct &sv,
                           sumpDynamicVariableStruct &dynamic) {

  volatile uint32_t* portDataInputRegister = &PORT_DATA_INPUT_REGISTER;
  volatile uint8_t* USB0_ISTAT_register = &USB0_ISTAT;
  uint32_t *inputPtr = sv.startOfBuffer;
  uint32_t tempValue = 0;
  uint32_t tempValue2 = 0;
  uint32_t workingValue = 0;

  if (sv.triggerMask)
  {
    sv.delaySamples = 1;

    set_led_on ();
  }

  maskInterrupts ();

  // assembly : "<instructions>: [] :: "rx");
  //    asm(code : output operand list : input operant list : clobber list);
  //
  //  output operand list = [result] "=r" (y)    result = name, y = c expression
  //        (= means write only, + means read/write, & means output only
  //  input operand list  = [value]  "r"  (x)    value = name, x = c expression

  asm volatile ("cmp %[trigMask],#0\n\t"
                "beq asm5_record_loop\n\t"

                //
                // look for trigger
                //

                // read sample
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "asm5_looking_for_trigger_loop:\n\t"

                // if (usbInterruptPending)
                "ldrb %[tempValue2],[%[USB0_ISTAT_register]]\n\t"
                "tst %[tempValue2], #251\n\t"
                "bne asm5_usb_exit\n\t"
                
                // save as first sample
                "mov %[workingValue],%[tempValue]\n\t"
                // read sample
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                // if ((tempValue & trigMask) == trigValue) {
                "and %[tempValue2],%[tempValue],%[trigMask]\n\t"
                "cmp %[tempValue2],%[trigValue]\n\t"
                "bne asm5_looking_for_trigger_loop\n\t"

                // trigger - save as second sample
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                // go to 3rd sample
                "b asm5_third_sample\n\t"

                //
                // record data
                //
                
                ".align 2\n\t"
                "asm5_record_loop:\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "asm5_loop:\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsls %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "asm5_third_sample:\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsls %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsls %[workingValue],%[workingValue],#8\n\t"
                "add %[tempValue2],%[workingValue],%[tempValue]\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "nop\n\t"
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]],#4\n\t"
                "nop\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[tempValue2],%[tempValue],%[workingValue],lsl #8\n\t"
                // if inputPtr >= endOfBuffer
                "cmp %[inputPtr],%[endOfBuffer]\n\t"
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]],#4\n\t"
                "nop\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "blt asm5_loop\n\t"
                "b asm5_exit\n\t"

                "asm5_usb_exit:\n\t"
                "mov %[USB0_ISTAT_register], #1\n\t"

                "asm5_exit:\n\t"

                : [endOfBuffer] "+l" (sv.endOfBuffer),
                  [inputPtr] "+l" (inputPtr),
                  [trigMask] "+r" (sv.triggerMask[0]),
                  [trigValue] "+r" (sv.triggerValue[0]),
                  [portDataInputRegister] "+l" (portDataInputRegister),
                  [USB0_ISTAT_register] "+l" (USB0_ISTAT_register),
                  [tempValue] "+l" (tempValue),
                  [tempValue2] "+l" (tempValue2),
                  [workingValue] "+l" (workingValue)
                :: "cc");

  unmaskInterrupts ();

  // Assembly sets the register address to 1 to indicate USB interrupt
  if ((uint32_t)USB0_ISTAT_register == 1)
  {
    DEBUG_SERIAL(print(" Halt due to USB interrupt"));
    set_led_off ();
    SUMPreset();
  }

  dynamic.triggerSampleIndex = 1;

  set_led_off ();
}
#else
// Teensy LC compiler has more constraints, so it takes 6 clocks. It has fewer registers available,
// so check for trigger using C++, then go to assembly
void recordDataAsm6Clocks (sumpSetupVariableStruct &sv,
                           sumpDynamicVariableStruct &dynamic) {

  volatile uint32_t* portDataInputRegister = &PORT_DATA_INPUT_REGISTER;
  uint32_t *inputPtr = sv.startOfBuffer;
  register uint32_t tempValue = 0;
  uint32_t tempValue2 = 0;
  uint32_t workingValue = 0;

  if (sv.triggerMask[0])
  {
    sv.delaySamples = 1;

    set_led_on ();
  }

  maskInterrupts ();

  // Look for trigger before starting assembly language.
  // This saves 2 samples - 1 before trigger and 1 at trigger.
  if (sv.triggerMask[0])
  {
    // read sample pre-trigger
    tempValue = PORT_DATA_INPUT_REGISTER;

    while (1)
    {
      workingValue = tempValue;

      // read sample
      tempValue = PORT_DATA_INPUT_REGISTER;

      // if trigger
      if ((tempValue & sv.triggerMask[0]) == sv.triggerValue[0])
      {
        // save as second sample
        workingValue = tempValue + (workingValue << 8);
        break;
      }

      // if USB interrupt pending
      if (USB0_ISTAT & ~USB_ISTAT_SOFTOK)
      {
        goto USB_Exit;
      }

    }
  }

  asm volatile ("cmp %[trigMask],#0\n\t"
                "bne asm5_third_sample\n\t"

                ".align 2\n\t"
                "asm5_record_loop:\n\t"
                "ldr %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "asm5_loop:\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "asm5_third_sample:\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "nop\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "nop\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[tempValue2],%[workingValue],%[tempValue]\n\t"
                "ldr %[workingValue], [%[portDataInputRegister],#0]\n\t"
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]]\n\t"
                "add %[inputPtr],%[inputPtr],#4\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "nop\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[workingValue],%[workingValue],%[tempValue]\n\t"
                "ldr %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "lsl %[workingValue],%[workingValue],#8\n\t"
                "add %[tempValue2],%[workingValue],%[tempValue]\n\t"
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]]\n\t"
                "add %[inputPtr],%[inputPtr],#4\n\t"
                "ldr %[workingValue], [%[portDataInputRegister],#0]\n\t"
                // if inputPtr >= endOfBuffer
                "cmp %[inputPtr],%[endOfBuffer]\n\t"
                "blt asm5_loop\n\t"
                "asm5_exit:\n\t"

                : [endOfBuffer] "+l" (sv.endOfBuffer),
                  [inputPtr] "+l" (inputPtr),
                  [trigMask] "+r" (sv.triggerMask[0]),
                  [portDataInputRegister] "+l" (portDataInputRegister),
                  [tempValue] "+l" (tempValue),
                  [tempValue2] "+l" (tempValue2),
                  [workingValue] "+l" (workingValue)
                :: "cc");

  unmaskInterrupts ();

  dynamic.triggerSampleIndex = 1;

  set_led_off ();

  return;

USB_Exit:
  unmaskInterrupts ();

  DEBUG_SERIAL(print(" Halt due to USB interrupt"));
  set_led_off ();
  SUMPreset();
}
#endif

void recordDataAsmWithTrigger (sumpSetupVariableStruct &sv,
                               sumpDynamicVariableStruct &dynamic) {

  // this uses 8 cpu cycles per sample

#if not Teensy_LC
  uint32_t *inputPtr = sv.startOfBuffer;

  volatile uint32_t* portDataInputRegister = &PORT_DATA_INPUT_REGISTER;
#if Teensy_4_0
  volatile uint32_t* USB0_ISTAT_register = &USB1_ENDPTCOMPLETE;
#else
  volatile uint8_t* USB0_ISTAT_register = &USB0_ISTAT;
#endif  
  uint32_t bufferHasWrapped = dynamic.bufferHasWrapped;
  uint32_t tempValue = 0;
  uint32_t workingValue = 0;
  uint32_t *startPtr = sv.startOfBuffer;
  uint32_t *startOfBuffer = sv.startOfBuffer;
  uint32_t *endOfBuffer = sv.endOfBuffer;


  byte samplesPerElement = sv.samplesPerElement;
  byte samplesPerElementMinusOne = samplesPerElement - 1;
  uint32_t *triggerPtr = startOfBuffer;

  uint32_t delaySizeBytes = sv.delaySizeInElements * 4;
  uint32_t bufferSizeBytes = (uint32_t)(endOfBuffer - startOfBuffer) * 4;

  // to preserve registers, pack delaySize and bufferSize into 1 int
  // (21 bits for delay size, 11 bits for buffer size (assume 1k increments)
  // This allows up to 2 Mb buffer sizes in 1k increments
  uint32_t data1 = (delaySizeBytes << 11) + (bufferSizeBytes >> 10);

  if (sv.triggerMask[0])
  {
    // position to arm the trigger
    startPtr = inputPtr + sv.delaySizeInElements;

    set_led_on ();
  }
  else
  {
    startPtr = endOfBuffer - 1;
  }

  maskInterrupts ();

  asm volatile ("cmp %[trigMask],#0\n\t"
                "beq record_loop\n\t"

  // read enough samples prior to arming to meet the pre-trigger request
  // while (1) {
                ".align 2\n\t"
                "pre_trigger_loop:\n\t"
    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
    // workingValue = (tempValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
             #if Teensy_3_6
                "nop\n\t"
             #endif
     // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
             #if Teensy_3_6
                "nop\n\t"
             #endif
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // *(inputPtr) = workingValue;
    // ++inputPtr;
                "str %[workingValue], [%[inputPtr]],4\n\t"
                 
    // if enough data is buffered
    // if (inputPtr >= startPtr) {
                "cmp %[inputPtr],%[startPtr]\n\t"
                "blt pre_trigger_loop\n\t"
      // move to armed state
      // break;
    // }

  // } // while (1)

  // while (1) {
    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#64\n\t"
             #if Teensy_3_5 or Teensy_3_6
                "nop\n\t"
                "nop\n\t"
             #endif
                "b looking_for_trigger_loop\n\t"
                "nop\n\t"
                ".align 2\n\t"
                "looking_for_trigger_loop:\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "nop\n\t"
    // if (usbInterruptPending) [part1]
               "ldrb %[tempValue],[%[USB0_ISTAT_register]]\n\t"
               "tst %[tempValue], #251\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // if (usbInterruptPending) [part2]
                "bne usb_exit\n\t"
//"orr %[workingValue],%[workingValue],#68\n\t"
                "nop\n\t"
                "nop\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // *(inputPtr) = workingValue;
    // ++inputPtr;
                "str %[workingValue], [%[inputPtr]],4\n\t"

    // adjust for circular buffer wraparound at the end
    // if (inputPtr >= endOfBuffer) {
                "cmp %[endOfBuffer],%[inputPtr]\n\t"
                "it ls\n\t"
      // inputPtr = startOfBuffer;
                "movls %[inputPtr],%[startOfBuffer]\n\t"
    // }

    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#64\n\t"

    // if trigger has occurred
    // if ((tempValue & trigMask) == trigValue) {
                "and %[tempValue],%[tempValue],%[trigMask]\n\t"
                "cmp %[tempValue],%[trigValue]\n\t"
                "bne looking_for_trigger_loop\n\t"

             #if Teensy_3_5 or Teensy_3_6
                "nop\n\t"
             #endif

      // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
//"orr %[tempValue],%[tempValue],#224\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

      // last location to save
      // startPtr = inputPtr - sv.delaySize;
               "sub %[startPtr],%[inputPtr],%[data1],lsr #11\n\t"

      // adjust for circular buffer wraparound at the end.
      // if (startPtr < startOfBuffer) {
                "cmp %[startPtr],%[startOfBuffer]\n\t"

             #if Teensy_3_5 or Teensy_3_6
                "nop\n\t"
             #endif

      // (read extra value to keep up)
      // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

                // get just lower 11 bits (plus 10 lower 0's)
                "lsl %[data1],#21\n\t"
                "it lo\n\t"
        // startPtr = startPtr + bufferSize;
               "addlo %[startPtr],%[startPtr],%[data1],lsr #11\n\t"
      // }

    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // *(inputPtr) = workingValue;
    // ++inputPtr;
                "str %[workingValue], [%[inputPtr]],4\n\t"

    // adjust for circular buffer wraparound at the end
    // if (inputPtr >= endOfBuffer) {
                "cmp %[endOfBuffer],%[inputPtr]\n\t"
                "it ls\n\t"
      // inputPtr = startOfBuffer;
                // .w for optimal timing alignment of record loop
                "movls.w %[inputPtr],%[startOfBuffer]\n\t"
    // }

      // break;
    // }
  // } // while (1)

 // } // if sumpTrigMask

                "record_loop:\n\t"
  // read samples after trigger
  // while (1) {

    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#128\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "b record_loop_second_sample\n\t"

                ".align 2\n\t"
                "record_loop_second_sample:\n\t"
    // workingValue = (workingValue << 8) + (tempValue = PORT_DATA_INPUT_REGISTER);
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

//                "cmp %[startOfBuffer],%[inputPtr]\n\t"
//                "it eq\n\t"
//"orreq %[workingValue],#8\n\t"
//       "movls %[inputPtr],%[startOfBuffer]\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // *(inputPtr) = workingValue;
    // ++inputPtr;
                "str %[workingValue], [%[inputPtr]],4\n\t"

    // adjust for circular buffer wraparound at the end
    // if (inputPtr >= endOfBuffer) {
                "cmp %[endOfBuffer],%[inputPtr]\n\t"
                "it ls\n\t"
      // inputPtr = startOfBuffer;
                "movls %[inputPtr],%[startOfBuffer]\n\t"
    // }

    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#128\n\t"
                "nop\n\t"
                "nop\n\t"
             #if Teensy_3_2 or Teensy_3_5 or Teensy_3_6
                "nop\n\t"
             #endif
    
    // if enough data is buffered
    // if (inputPtr == startPtr) {
                "cmp %[inputPtr],%[startPtr]\n\t"
                "bne record_loop_second_sample\n\t"
      // move to armed state
      // break;
    // }
  // } // while (1)

                "b good_exit\n\t"

                "usb_exit:\n\t"
                "mov %[USB0_ISTAT_register], #1\n\t"

                "good_exit:\n\t"
                : [endOfBuffer] "+l" (sv.endOfBuffer),
                  [inputPtr] "+l" (inputPtr),
                  [startPtr] "+l" (startPtr),
                  [trigMask] "+r" (sv.triggerMask[0]),
                  [trigValue] "+r" (sv.triggerValue[0]),
                  [portDataInputRegister] "+l" (portDataInputRegister),
                  [USB0_ISTAT_register] "+l" (USB0_ISTAT_register),
                  [startOfBuffer] "+r" (sv.startOfBuffer),
                  [data1] "+r" (data1),
                  [tempValue] "+l" (tempValue),
                  [workingValue] "+l" (workingValue)
                :: "cc");

  unmaskInterrupts ();

  // Assembly sets the register address to 1 to indicate USB interrupt
  if ((uint32_t)USB0_ISTAT_register == 1)
  {
    DEBUG_SERIAL(print(" Halt due to USB interrupt"));
    set_led_off ();
    SUMPreset();
  }

  if (sv.triggerMask[0])
  {
    triggerPtr = startPtr + sv.delaySizeInElements;

    // if using trigger, it always wraps
    bufferHasWrapped = true;
  }

  // adjust trigger count
  dynamic.triggerSampleIndex = (triggerPtr - startOfBuffer) * samplesPerElement;

  // inputPtr is always incremented, even if first sample is the trigger
  if (bufferHasWrapped && (inputPtr != (startOfBuffer + 1)))
  {
    dynamic.bufferHasWrapped = true;
  }

  set_led_off ();

#endif
}

void recordDataAsm3Clocks (sumpSetupVariableStruct &sv,
                           sumpDynamicVariableStruct &dynamic) {

#if not Teensy_LC

  // This function records data in 1k byte chunks. At the end of each loop,
  // it decrements a counter and jumps to the top. This takes an extra 3 clocks,
  // so it loses 1 sample for every 1k recorded.
  
  volatile uint32_t* portDataInputRegister = &PORT_DATA_INPUT_REGISTER;
  volatile uint8_t* USB0_ISTAT_register = &USB0_ISTAT;
  boolean fillingCache = false;
  uint32_t *inputPtr;
  uint32_t loopCount = 1;
  uint32_t tempValue = 0;
  byte triggerMask = sv.triggerMask[0];
  uint32_t workingValue = 0;

  // need to record in upper SRAM (which starts at 0x2000_0000) to get the speed
  sv.startOfBuffer = (uint32_t *)0x20000000;

  // assumes 4 samples per element
  dynamic.triggerSampleIndex = 0;

  sv.samplesToRecord = (sv.endOfMemory - sv.startOfBuffer) * 4;

DEBUG_SERIAL(print("  endOfMemory: "));
DEBUG_SERIAL(print((uint32_t)sv.endOfMemory, HEX));
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print("  samplesToRecord: "));
DEBUG_SERIAL(print((uint32_t)sv.samplesToRecord));
DEBUG_SERIAL(println(""));
  if (sv.samplesToRecord > sv.samplesToSend) {
    sv.samplesToRecord = sv.samplesToSend;
  }

  loopCount = sv.samplesToRecord / 1024;
  sv.samplesToRecord = loopCount * 1024;
  sv.samplesToSend = sv.samplesToRecord;
  sv.endOfBuffer = sv.startOfBuffer + (sv.samplesToSend / 4);
  inputPtr = sv.startOfBuffer;

DEBUG_SERIAL(print("  // inputPtr: "));
DEBUG_SERIAL(print((uint32_t)inputPtr, HEX));
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print("  startOfBuffer: "));
DEBUG_SERIAL(print((uint32_t)sv.startOfBuffer, HEX));
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print("  endOfBuffer: "));
DEBUG_SERIAL(print((uint32_t)sv.endOfBuffer, HEX));
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print("  loopCount: "));
DEBUG_SERIAL(print((uint32_t)loopCount));
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print("  samplesToSend: "));
DEBUG_SERIAL(print((uint32_t)sv.samplesToSend));
DEBUG_SERIAL(println(""));

delay(500);

#if Teensy_3_6
  // call loop once to get the code in the cache so it runs faster
  uint32_t *previousInputPtr = inputPtr;
  int previousLoopCount = loopCount;

  // set up for one loop, no trigger
  loopCount = 1;
  fillingCache = true;
  triggerMask = 0;

  // call the loop and return to here
  goto RunAsmLoop;

  ReturnFromFillingCache:

  // restore data
  fillingCache = false;
  inputPtr = previousInputPtr;
  loopCount = previousLoopCount;
  triggerMask = sv.triggerMask[0];
#endif

  if (sv.triggerMask[0])
  {
    // trigger is always at the beginning
    sv.delaySamples = 0;

    set_led_on ();
  }

  maskInterrupts ();

  // assembly : "<instructions>: [] :: "rx");
  //    asm(code : output operand list : input operant list : clobber list);
  //
  //  output operand list = [result] "=r" (y)    result = name, y = c expression
  //        (= means write only, + means read/write, & means output only
  //  input operand list  = [value]  "r"  (x)    value = name, x = c expression

  // because of endianness (Teensy is little endian), need to store bytes in
  // locations 3,2,1,0, 7,6,5,4, etc. This is so that send_data can pull the data
  // out as 32-bit integers.

  RunAsmLoop:
  asm volatile ("add %[inputPtr],%[inputPtr],#3\n\t"
                "cmp %[trigMask],#0\n\t"
                "beq asm3_record_loop\n\t"

                //
                // look for trigger
                //

                // read sample
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "asm3_looking_for_trigger_loop:\n\t"

                // if (usbInterruptPending)
                "ldrb %[tempValue],[%[USB0_ISTAT_register]]\n\t"
                "tst %[tempValue], #251\n\t"
                "bne asm3_usb_exit\n\t"
                
                // save as first sample but don't increment inputPtr yet
                "strb %[workingValue], [%[inputPtr]]\n\t"
                // read sample
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                // if ((workingValue & trigMask) == trigValue) {
                "and %[tempValue],%[workingValue],%[trigMask]\n\t"
                "cmp %[tempValue],%[trigValue]\n\t"
                "bne asm3_looking_for_trigger_loop\n\t"

                // trigger - save as second sample
                "sub %[inputPtr],%[inputPtr],#1\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"

                // go to 3rd sample
                "b asm3_third_sample\n\t"

                //
                // record data
                //
                
                ".align 3\n\t"
                "asm3_record_loop:\n\t"

                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "asm3_third_sample:\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 64
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 128
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 192
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 256

                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 320
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 384

                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 64
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 512
                
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 64
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 640
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 192
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 768

                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 320
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 896

                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 32
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 48
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 64
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 16
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#-1\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "strb %[workingValue], [%[inputPtr]],#7\n\t"
// 1024
                "subs %[loopCount], %[loopCount],#1\n\t"
                "bne asm3_record_loop\n\t"

                "b asm3_exit\n\t"

                "asm3_usb_exit:\n\t"
                "mov %[USB0_ISTAT_register], #1\n\t"

                "asm3_exit:\n\t"

                : [inputPtr] "+l" (inputPtr),
                  [trigMask] "+r" (triggerMask),
                  [trigValue] "+r" (sv.triggerValue[0]),
                  [portDataInputRegister] "+l" (portDataInputRegister),
                  [USB0_ISTAT_register] "+l" (USB0_ISTAT_register),
                  [tempValue] "+l" (tempValue),
                  [loopCount] "+l" (loopCount),
                  [workingValue] "+l" (workingValue)
                :: "cc");

  unmaskInterrupts ();

#if Teensy_3_6
  if (fillingCache) {
    goto ReturnFromFillingCache;
  }
#endif

  // Assembly sets the register address to 1 to indicate USB interrupt
  if ((uint32_t)USB0_ISTAT_register == 1)
  {
    DEBUG_SERIAL(print(" Halt due to USB interrupt"));
    set_led_off ();
    SUMPreset();
  }

  set_led_off ();

#endif
}

#endif  // if not Teensy 4_0
