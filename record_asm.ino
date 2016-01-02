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

                // read sample
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "asm5_looking_for_trigger_loop:\n\t"
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
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]],4\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"
                "ldrb %[tempValue], [%[portDataInputRegister],#0]\n\t"
                "add %[tempValue2],%[tempValue],%[workingValue],lsl #8\n\t"
                // if inputPtr >= endOfBuffer
                "cmp %[inputPtr],%[endOfBuffer]\n\t"
                // store 4 samples
                "str %[tempValue2], [%[inputPtr]],4\n\t"
                "ldrb %[workingValue], [%[portDataInputRegister],#0]\n\t"
                "blt asm5_loop\n\t"
                "asm5_exit:\n\t"

                : [endOfBuffer] "+l" (sv.endOfBuffer),
                  [inputPtr] "+l" (inputPtr),
                  [trigMask] "+r" (sv.triggerMask),
                  [trigValue] "+r" (sv.triggerValue),
                  [portDataInputRegister] "+l" (portDataInputRegister),
                  [USB0_ISTAT_register] "+l" (USB0_ISTAT_register),
                  [tempValue] "+l" (tempValue),
                  [tempValue2] "+l" (tempValue2),
                  [workingValue] "+l" (workingValue)
                :: "cc");

  unmaskInterrupts ();

  dynamic.triggerSampleIndex = 1;

DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print(", inputPtr: "));
DEBUG_SERIAL(print((int)inputPtr, HEX));
DEBUG_SERIAL(print(", dsb: "));
DEBUG_SERIAL(println(""));
delay (100);
}

void recordDataAsmWithTrigger (sumpSetupVariableStruct &sv,
                               sumpDynamicVariableStruct &dynamic) {

  uint32_t *inputPtr = sv.startOfBuffer;

  volatile uint32_t* portDataInputRegister = &PORT_DATA_INPUT_REGISTER;
//  volatile uint32_t* portDataInputRegister = &SYST_CVR;
  volatile uint8_t* USB0_ISTAT_register = &USB0_ISTAT;
  uint32_t tempValue = 0;
  uint32_t workingValue = 0;
  uint32_t *startPtr = sv.startOfBuffer;
  uint32_t *startOfBuffer = sv.startOfBuffer;
  uint32_t *endOfBuffer = sv.endOfBuffer;


  byte samplesPerElement = sv.samplesPerElement;
  byte samplesPerElementMinusOne = samplesPerElement - 1;
  int triggerCount = samplesPerElementMinusOne;
  uint32_t *triggerPtr = startOfBuffer;

  uint32_t delaySizeBytes = sv.delaySizeInElements * 4;
  uint32_t bufferSizeBytes = (uint32_t)(endOfBuffer - startOfBuffer) * 4;

  // to preserve registers, pack delaySize and bufferSize into 1 int
  uint32_t data1 = (delaySizeBytes << 16) + bufferSizeBytes;

  uint32_t data2 = (delaySizeBytes << 16) + bufferSizeBytes;
  asm volatile ("lsr %[data2],#16\n\t" : [data2] "+r" (data2));
  
DEBUG_SERIAL(println(""));
DEBUG_SERIAL(print(", startPtr: "));
DEBUG_SERIAL(print((int)startPtr, HEX));
DEBUG_SERIAL(print(", dsb: "));
DEBUG_SERIAL(print((int)delaySizeBytes, HEX));
DEBUG_SERIAL(print(", bsb: "));
DEBUG_SERIAL(print((int)bufferSizeBytes, HEX));
DEBUG_SERIAL(print(", data1: "));
DEBUG_SERIAL(print((int)data1, HEX));
DEBUG_SERIAL(print(", data2: "));
DEBUG_SERIAL(print((int)data2, HEX));
DEBUG_SERIAL(println(""));
delay (100);
  
  if (sv.triggerMask)
  {
    // position to arm the trigger
    startPtr = inputPtr + sv.delaySizeInElements;
  }
  else
  {
    startPtr = endOfBuffer - 1;
  }

   maskInterrupts ();

// if (sumpTrigMask){
  asm volatile ("cmp %[trigMask],#0\n\t"
                "beq record_loop\n\t"

  // read enough samples prior to arming to meet the pre-trigger request
  // while (1) {
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
                 
    // if enough data is buffered
    // if (inputPtr >= startPtr) {
                "cmp %[inputPtr],%[startPtr]\n\t"
                "blt pre_trigger_loop\n\t"
////                "nop\n\t"
      // move to armed state
      // break;
    // }

  // } // while (1)

  // while (1) {
    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#64\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "looking_for_trigger_loop:\n\t"
////                "nop\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // if (usbInterruptPending) [part1]
"ldrb %[tempValue],[%[USB0_ISTAT_register]]\n\t"
"tst %[tempValue], #251\n\t"
//                "nop\n\t"
//                "nop\n\t"
//                "nop\n\t"
//                "nop\n\t"
    // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

    // if (usbInterruptPending) [part2]
                "bne usb_exit\n\t"
//                "nop\n\t"
//                "nop\n\t"
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

      // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
//"orr %[tempValue],%[tempValue],#224\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

      // last location to save
      // startPtr = inputPtr - sv.delaySize;
//                "sub %[startPtr],%[inputPtr],%[delaySize],lsl #2\n\t"
  "sub %[startPtr],%[inputPtr],%[data1],lsr #16\n\t"

      // adjust for circular buffer wraparound at the end.
      // if (startPtr < startOfBuffer) {
                "cmp %[startPtr],%[startOfBuffer]\n\t"

// (read extra value to keep up)
      // workingValue = (workingValue << 8) + PORT_DATA_INPUT_REGISTER;
                "ldrb %[tempValue], [%[portDataInputRegister]]\n\t"
                "add %[workingValue],%[tempValue],%[workingValue],lsl #8\n\t"

// get just lower 16 bits
"lsl %[data1],#16\n\t"
                "it ls\n\t"
        // startPtr = startPtr + bufferSize;
//                "addls %[startPtr],%[startPtr],%[bufferSize],lsl #2\n\t"
  "addls %[startPtr],%[startPtr],%[data1],lsr #16\n\t"
      // }

//                "nop\n\t"
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
//                "nop\n\t"
//                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"
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

                "nop\n\t"
    // workingValue = PORT_DATA_INPUT_REGISTER;
                "ldrb %[workingValue], [%[portDataInputRegister]]\n\t"
//"orr %[workingValue],%[workingValue],#128\n\t"
                "nop\n\t"
                "nop\n\t"
                "nop\n\t"

    
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
                  [trigMask] "+r" (sv.triggerMask),
                  [trigValue] "+r" (sv.triggerValue),
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

  if (sv.triggerMask)
  {
      triggerCount = 3;
      triggerPtr = startPtr + sv.delaySizeInElements;
  }
      
  // adjust trigger count
  dynamic.triggerSampleIndex = (triggerPtr - startOfBuffer) * samplesPerElement + samplesPerElementMinusOne - triggerCount;

DEBUG_SERIAL(print(", startPtr: "));
DEBUG_SERIAL(print((int)startPtr, HEX));
DEBUG_SERIAL(print(", triggerPtr: "));
DEBUG_SERIAL(print((int)triggerPtr, HEX));
DEBUG_SERIAL(print(", triggerSampleIndex: "));
DEBUG_SERIAL(print((int)dynamic.triggerSampleIndex, HEX));
DEBUG_SERIAL(println(""));
delay (100);

}
