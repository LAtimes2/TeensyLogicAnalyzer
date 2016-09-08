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

// Resources used:
//   PIT timer 0
//   Port D pins or SPI MISO pins
//   Serial : interface to SUMP user interface
//   Serial1/2/3 : debug info (if turned on)

//
//  User Configuration settings
//

#define HARDWARE_CONFIGURATION 0   // if 1, it will use the SPI input(s) instead of Port D
#define CREATE_TEST_FREQUENCIES 0  // if 1, it will output frequencies on pins 5/6/0/21 (channels 4-7)

//
// Pin definitions (info only - do not change)
//

#if HARDWARE_CONFIGURATION

#define CHAN0 1   // or 26 (3.1) / 5 (LC)
#define CHAN1 8   // or 12

#else

#define CHAN0 2
#define CHAN1 14
#define CHAN2 7
#define CHAN3 8
#define CHAN4 6
#define CHAN5 20
#define CHAN6 21
#define CHAN7 5

#endif

#define LED_PIN 13

#define TIMING_PIN_0 15
#define TIMING_PIN_1 16

//////////////////////////////////////
// End of settings
//////////////////////////////////////

#include <stdint.h>
#include "types.h"

#define VERSION "3.2"

//#define TIMING_DISCRETES  // if uncommented, set pins for timing

// Debug serial port. Uncomment one of these lines
#define DEBUG_SERIAL(x) 0   // no debug output
//#define DEBUG_SERIAL(x) Serial.x  // debug output to USB Serial
//#define DEBUG_SERIAL(x) Serial1.x // debug output to Serial1
//#define DEBUG_SERIAL(x) Serial2.x // debug output to Serial2
//#define DEBUG_SERIAL(x) Serial3.x // debug output to Serial3

// define the various types of Teensy's

// Teensy 3.0
#if defined(__MK20DX128__)
  #define Teensy_3_0 1
// Teensy 3.2
#elif defined(__MK20DX256__)
  #define Teensy_3_2 1
// Teensy LC
#elif defined(__MKL26Z64__)
  #define Teensy_LC 1
// Teensy 3.5
#elif defined(__MK64FX512__)
  #define Teensy_3_5 1
// Teensy 3.6
#elif defined(__MK66FX1M0__)
  #define Teensy_3_6 1
#endif

#if Teensy_3_0

   // 12k buffer size
   #define LA_SAMPLE_SIZE 12 * 1024

#elif Teensy_3_2
   // 58k buffer size
   #define LA_SAMPLE_SIZE 58 * 1024

#elif Teensy_LC

   // 4k buffer size
   #define LA_SAMPLE_SIZE 4 * 1024

#elif Teensy_3_5
   // 186k buffer size
   #define LA_SAMPLE_SIZE 186 * 1024

#elif Teensy_3_6
   // 250k buffer size
   #define LA_SAMPLE_SIZE 250 * 1024

#else

   // 4k buffer size
   #define LA_SAMPLE_SIZE 4 * 1024

#endif

// use Port D for sampling
#define PORT_DATA_INPUT_REGISTER  GPIOD_PDIR

// PIT timer registers
#define TIMER_CONTROL_REGISTER    PIT_TCTRL0
#define TIMER_FLAG_REGISTER       PIT_TFLG0
#define TIMER_LOAD_VALUE_REGISTER PIT_LDVAL0

// SUMP protocol values
/* XON/XOFF are not supported. */
#define SUMP_RESET 0x00
#define SUMP_RUN   0x01
#define SUMP_ID    0x02
#define SUMP_DESC  0x04
#define SUMP_XON   0x11
#define SUMP_XOFF  0x13
#define SUMP_DIV   0x80
#define SUMP_CNT   0x81
#define SUMP_FLAGS 0x82
#define SUMP_CAPTURE_DELAY 0x83  // from pipistrello
#define SUMP_CAPTURE_COUNT 0x84  // from pipistrello
#define SUMP_TRIG_1_MASK   0xC0
#define SUMP_TRIG_1_VALS   0xC1
#define SUMP_TRIG_1_CONFIG 0xC2
#define SUMP_TRIG_2_MASK   0xC4
#define SUMP_TRIG_2_VALS   0xC5
#define SUMP_TRIG_2_CONFIG 0xC6
#define SUMP_TRIG_3_MASK   0xC8
#define SUMP_TRIG_3_VALS   0xC9
#define SUMP_TRIG_3_CONFIG 0xCA
#define SUMP_TRIG_4_MASK   0xCC
#define SUMP_TRIG_4_VALS   0xCD
#define SUMP_TRIG_4_CONFIG 0xCE

// DEBUG_SERIAL 0 causes this warning. It is not a usual warning, so ignoring it is low risk
#pragma GCC diagnostic ignored "-Wunused-value"

// this is the main data storage array. Add 20 extra bytes
// just in case (triggering may use up to 16 extra).
// Needs to be aligned to a 4 byte boundary
byte logicData[LA_SAMPLE_SIZE + 20] __attribute__ ((aligned));

enum strategyType {
  STRATEGY_NORMAL,
  STRATEGY_PACKED,
  STRATEGY_ASSEMBLY,
} sumpStrategy = STRATEGY_NORMAL;

int currentFBUS = F_BUS;
uint32_t sumpRunning = 0;

uint32_t blinkStartTime = 0;
uint32_t blinkEndTime = 0;
uint32_t previousBlinkTime = 0;

// working data for a 5-byte command from the SUMP Interface
struct _sumpRX {
  byte command[5];
  byte parameters;
  byte parCnt;
} sumpRX;

// for SUMP command state machine
enum _SUMP {
  C_IDLE = 0,
  C_PARAMETERS,
} sumpRXstate = C_IDLE;

// Forward declarations
void recordDataAsm5Clocks (sumpSetupVariableStruct &sv,
                           sumpDynamicVariableStruct &dynamic);
void recordDataAsm6Clocks (sumpSetupVariableStruct &sv,
                           sumpDynamicVariableStruct &dynamic);
void recordDataAsmWithTrigger (sumpSetupVariableStruct &sv,
                               sumpDynamicVariableStruct &dynamic);
void recordLowSpeedData (sumpSetupVariableStruct &sv,
                         sumpDynamicVariableStruct &dynamic);
void recordSPIData (sumpSetupVariableStruct &sv,
                    sumpDynamicVariableStruct &dynamic);
void sendData (sumpSetupVariableStruct sumpSetup,
               sumpDynamicVariableStruct dynamic);

void SUMPprocessCommands(byte inByte,
                         struct sumpSetupVariableStruct &sumpSetup);

void processSingleByteCommand (byte inByte,
                               struct sumpSetupVariableStruct &sumpSetup);
void processFiveByteCommand (byte command[],
                             struct sumpSetupVariableStruct &sumpSetup);

void setup()
{
  DEBUG_SERIAL(begin (921600));  // set baud rate
  DEBUG_SERIAL(println(""));
  DEBUG_SERIAL(println("Logic Analyzer"));

#if HARDWARE_CONFIGURATION
  // set up pins to record data on
  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);

#else
  // set up pins to record data on
  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);
  pinMode(CHAN2, INPUT);
  pinMode(CHAN3, INPUT);
  pinMode(CHAN4, INPUT);
  pinMode(CHAN5, INPUT);
  pinMode(CHAN6, INPUT);
  pinMode(CHAN7, INPUT);

#endif

  pinMode(LED_PIN, OUTPUT);

#ifdef TIMING_DISCRETES
  // Pin 0 high = waiting to sample, Pin 1 high = looking for trigger
  pinMode (TIMING_PIN_0, OUTPUT);
  pinMode (TIMING_PIN_1, OUTPUT);

#endif

  blinkled();

  setupTestFrequencies (F_BUS);

  SUMPreset();
}

void setupTestFrequencies (int newFBUS) {

currentFBUS = newFBUS;

#if CREATE_TEST_FREQUENCIES

  /* Use PWM to generate a test signal.
   *  If set on one of the analyzer pins, it will show up when recording.
   *  To test other pins, jumper a PWM output to an analyzer input.
   */

  // PWM available on pins 3-6,9,10,20-23
  // Port D: chan 4(6),5(20),6(21),7(5)

  // some modes change F_BUS, so this accounts for that
  int multiplier = 1;
  int divider = 1;

  float FBUSRatio = (float)currentFBUS / (float)F_BUS;

  if (FBUSRatio > 3.99) {
    divider = 4;
  }
  else if (FBUSRatio > 2.99) {
    divider = 3;
  }
  else if (FBUSRatio > 1.99) {
    divider = 2;
  }
  else if (FBUSRatio > 1.49) {
    multiplier = 2;
    divider = 3;
  }
  
  analogWriteFrequency (3, 62500 * multiplier / divider);
  analogWrite (3, 128);

  #if not HARDWARE_CONFIGURATION

    analogWriteFrequency (CHAN4, 25000 * multiplier / divider);
    analogWrite (CHAN4, 64);
    analogWrite (CHAN5, 124);

    #if defined(KINETISK)
      analogWrite (CHAN6, 128);
      analogWrite (CHAN7, 192);
    #endif
  #endif
#endif

}

int getCurrentFBUS () {
  return currentFBUS;
}

// main loop
void loop()
{
  byte inByte;
  struct sumpSetupVariableStruct sumpSetup;

  // set to no trigger initially
  sumpSetup.triggerMask[0] = 0;
  sumpSetup.lastTriggerLevel = 0;

  // loop forever
  while (1) {

    // NOTE: to provide quick response time (needed by sigrok), do not
    //       call blinkLED within this loop. Use startBlink instead.

    // check for input from SUMP interface
    if (Serial.available() > 0) {
      // blink LED when command is received
      startBlinkLEDFast();

      // read and process a byte from serial port
      inByte = Serial.read();

      // write command bytes to debug port
      DEBUG_SERIAL(print(inByte, HEX));
      DEBUG_SERIAL(print(","));

      SUMPprocessCommands(inByte, sumpSetup);
    }

    // check for input from Debug port
    if (DEBUG_SERIAL(available()) > 0) {
      startBlinkLEDFast();
      SUMPprocessCommands(DEBUG_SERIAL(read()), sumpSetup);
    }

    // if commanded to start, then record data
    if (sumpRunning) {
      SUMPrecordData(sumpSetup);
    }

    // blink LED every 2 seconds if not recording
    if ((millis() - previousBlinkTime) > 2000) {
      previousBlinkTime = millis();
      startBlinkLED();
    }

    // check if time to turn off LED
    checkBlinkLED();
  }
}

void SUMPreset(void) {

  sumpRunning = 0;
}

void SUMPprocessCommands(byte inByte,
                         sumpSetupVariableStruct &sumpSetup)
{
  switch (sumpRXstate) { //this is a state machine that grabs the incoming commands one byte at a time

    case C_IDLE:

      processSingleByteCommand (inByte, sumpSetup);
      break;

    case C_PARAMETERS:
      sumpRX.parCnt++;
      sumpRX.command[sumpRX.parCnt] = inByte; // store each parameter

      // if all parameters received
      if (sumpRX.parCnt == sumpRX.parameters)
      {
        processFiveByteCommand (sumpRX.command, sumpSetup);

        sumpRXstate = C_IDLE;
      }  
      break;

    default:
      blinkled();
      sumpRXstate = C_IDLE;
      break;
  }
}

void processSingleByteCommand (byte inByte,
                               sumpSetupVariableStruct &sumpSetup){

  switch (inByte) { // switch on the current byte

    case SUMP_RESET: // reset
      SUMPreset();
      break;

    case SUMP_ID: // SLA0 or 1 backwards: 1ALS
    // special command for debugging
    case '3':
      DEBUG_SERIAL(println ("SUMP_ID"));

      Serial.write("1ALS");
      break;

    case SUMP_RUN: // arm the trigger
      set_led_on (); // ARMED, turn on LED

      // tell data recorder to start
      sumpRunning = 1;
      break;

    case SUMP_DESC:
    // special command for debugging
    case '2':
      DEBUG_SERIAL(println ("SUMP_DESC"));

      // device name string
      Serial.write(0x01);
#if CREATE_TEST_FREQUENCIES
      Serial.write("Demo");
#endif
#if HARDWARE_CONFIGURATION
      Serial.write("HW_");
#endif
      if (F_CPU == 96000000) {
        Serial.write("Teensy96");
      } else if (F_CPU == 120000000) {
        #if defined(TEENSY_3_5)
          Serial.write("Teensy35_120");
        #else
          Serial.write("Teensy120");
        #endif
      } else if (F_CPU == 144000000) {
        Serial.write("Teensy144");
      } else if (F_CPU == 240000000) {
        Serial.write("Teensy36_240");
      } else if (F_CPU == 72000000) {
        Serial.write("Teensy72");
      } else if (F_CPU == 48000000) {
        #if defined(KINETISL)
          Serial.write("TeensyLC48");
        #else
          Serial.write("Teensy48");
        #endif
      } else {
        Serial.write("Teensy");
      }
      Serial.write(0x00);
      // firmware version string
      Serial.write(0x02);
      Serial.write(VERSION);
      Serial.write(0x00);
      // sample memory (4096)
      Serial.write(0x21);
      Serial.write(0x00);
      Serial.write(0x00);
      Serial.write(0x10);
      Serial.write(0x00);
      // sample rate (1MHz)
      Serial.write(0x23);
      Serial.write(0x00);
      Serial.write(0x0F);
      Serial.write(0x42);
      Serial.write(0x40);
      // number of probes (8)
      Serial.write(0x40);
      Serial.write(0x08);
      // protocol version (2)
      Serial.write(0x41);
      Serial.write(0x02);
      Serial.write(0x00);
      break;

    case SUMP_XON: // resume send data
      //   xflow=1;
      break;

    case SUMP_XOFF: // pause send data
      //   xflow=0;
      break;

    // special commands for debugging
    case '1':
      DEBUG_SERIAL(write ("samplesToRecord: "));
      DEBUG_SERIAL(println (sumpSetup.samplesToRecord));
      DEBUG_SERIAL(write ("sumpRequestedSamples: "));
      DEBUG_SERIAL(println (sumpSetup.samplesRequested));
      DEBUG_SERIAL(write ("sumpSetup.busClockDivisor: "));
      DEBUG_SERIAL(println (sumpSetup.busClockDivisor, HEX));
      DEBUG_SERIAL(write ("sumpSetup.clockFrequency: "));
      DEBUG_SERIAL(println (sumpSetup.clockFrequency));
      DEBUG_SERIAL(write ("sumpSetup.cpuClockTicks: "));
      DEBUG_SERIAL(println (sumpSetup.cpuClockTicks));
      break;

    case 'r':
      sumpSetup.samplesToRecord = 2048;
      sumpSetup.samplesRequested = sumpSetup.samplesToRecord;
      sumpSetup.delaySamples = 0;

      sumpSetup.busClockDivisor = 7;

      sumpSetup.clockFrequency = F_BUS / (sumpSetup.busClockDivisor + 1);
      sumpSetup.cpuClockTicks = F_CPU / sumpSetup.clockFrequency;

      set_led_on (); // ARMED, turn on LED

      // tell data recorder to start
      sumpRunning = 1;
      break;

    // special command for debugging
    case '5':
/* uses lots of RAM
      Serial.write ("logicData: ");
      Serial.print (logicData[0], HEX);
      Serial.print (", ");
      Serial.print (logicData[1], HEX);
      Serial.print (", ");
      Serial.print (logicData[2], HEX);
      Serial.print (", ");
      Serial.print (logicData[3], HEX);
      Serial.print (", ");
      Serial.print (logicData[4], HEX);
      Serial.print (", ");
      Serial.print (logicData[5], HEX);
      Serial.print (", ");
      Serial.print (logicData[6], HEX);
      Serial.print (", ");
      Serial.println (logicData[7], HEX);
      Serial.write ("logicData (32): ");
      Serial.println ((uint32_t)(*(uint32_t *)logicData), HEX);
*/
      break;

    // <cr>, <lf> - ignore
    case 0x0D:
    case 0x0A:
      break;

    default: // 5-byte command

      sumpRX.command[0] = inByte; // store first command byte
      sumpRX.parameters = 4; // all long commands are 5 bytes, get 4 parameters
      sumpRX.parCnt = 0; // reset the parameter counter
      sumpRXstate = C_PARAMETERS;
      break;
  }
}

void processFiveByteCommand (byte command[],
                             sumpSetupVariableStruct &sumpSetup)
{
  uint32_t divisor;

  switch (sumpRX.command[0]) {

    case SUMP_TRIG_1_MASK: // mask for bits to trigger on
      sumpSetup.triggerMask[0] = sumpRX.command[1];
      break;
    case SUMP_TRIG_2_MASK:
      sumpSetup.triggerMask[1] = sumpRX.command[1];
      break;
    case SUMP_TRIG_3_MASK:
      sumpSetup.triggerMask[2] = sumpRX.command[1];
      break;
    case SUMP_TRIG_4_MASK:
      sumpSetup.triggerMask[3] = sumpRX.command[1];
      break;

    case SUMP_TRIG_1_VALS: // value to trigger on
      sumpSetup.triggerValue[0] = sumpRX.command[1];
      // reset to invalid value
      sumpSetup.lastTriggerLevel = -1;
      break;
    case SUMP_TRIG_2_VALS:
      sumpSetup.triggerValue[1] = sumpRX.command[1];
      break;
    case SUMP_TRIG_3_VALS:
      sumpSetup.triggerValue[2] = sumpRX.command[1];
      break;
    case SUMP_TRIG_4_VALS:
      sumpSetup.triggerValue[3] = sumpRX.command[1];
      break;

    case SUMP_TRIG_1_CONFIG: // trigger configuration
      sumpSetup.triggerDelay[0] = sumpRX.command[1] + (sumpRX.command[2] << 8);
      // set lastTriggerLevel to the first stage with the Capture bit set
      if ((sumpRX.command[4] & 0x08) != 0 && sumpSetup.lastTriggerLevel == -1) {
        sumpSetup.lastTriggerLevel = 0;
      }
      break;
    case SUMP_TRIG_2_CONFIG:
      sumpSetup.triggerDelay[1] = sumpRX.command[1] + (sumpRX.command[2] << 8);
      if ((sumpRX.command[4] & 0x08) != 0 && sumpSetup.lastTriggerLevel == -1) {
        sumpSetup.lastTriggerLevel = 1;
      }
      break;
    case SUMP_TRIG_3_CONFIG:
      sumpSetup.triggerDelay[2] = sumpRX.command[1] + (sumpRX.command[2] << 8);
      if ((sumpRX.command[4] & 0x08) != 0 && sumpSetup.lastTriggerLevel == -1) {
        sumpSetup.lastTriggerLevel = 2;
      }
      break;
    case SUMP_TRIG_4_CONFIG:
      sumpSetup.triggerDelay[3] = sumpRX.command[1] + (sumpRX.command[2] << 8);
      if ((sumpRX.command[4] & 0x08) != 0 && sumpSetup.lastTriggerLevel == -1) {
        sumpSetup.lastTriggerLevel = 3;
      }
      break;

    case SUMP_CNT:
      sumpSetup.samplesToRecord = sumpRX.command[2];
      sumpSetup.samplesToRecord <<= 8;
      sumpSetup.samplesToRecord |= sumpRX.command[1];
      sumpSetup.samplesToRecord = (sumpSetup.samplesToRecord + 1) * 4;

      sumpSetup.delaySamples = sumpRX.command[4];
      sumpSetup.delaySamples <<= 8;
      sumpSetup.delaySamples |= sumpRX.command[3];
      sumpSetup.delaySamples = (sumpSetup.delaySamples + 1) * 4;

      // need this to get OLS client to line up trigger to time 0
      sumpSetup.delaySamples += 2;

      // command only supports up to 256k, but using certain assumptions, can go higher
      // (for clients that don't support SUMP_CAPTURE_COUNT command)
      if (sumpSetup.samplesToRecord == 208 * 1024)
      {
        // 464 is sent as 208 due to 256k wraparound
        sumpSetup.samplesToRecord = 464 * 1024;
      }

      break;

    case SUMP_DIV:
      divisor = sumpRX.command[3];
      divisor <<= 8;
      divisor |= sumpRX.command[2];
      divisor <<= 8;
      divisor |= sumpRX.command[1];

      // by setting device.dividerClockspeed = F_BUS, divisor is exactly equal to value to put in timer
      sumpSetup.busClockDivisor = divisor;

      sumpSetup.clockFrequency = F_BUS / (divisor + 1);

      if (F_CPU == 144000000) {
        // increase Bus clock by 50% and Mem clock
        int currentFBUS = 72000000;

        // set F_BUS equal to F_CPU / 2
        SIM_CLKDIV1 = (SIM_CLKDIV1 & ~SIM_CLKDIV1_OUTDIV2(0x0F)) | SIM_CLKDIV1_OUTDIV2(1);
        // set F_MEM to F_CPU / 4
        SIM_CLKDIV1 = (SIM_CLKDIV1 & ~SIM_CLKDIV1_OUTDIV4(0x0F)) | SIM_CLKDIV1_OUTDIV4(3);

        sumpSetup.clockFrequency = currentFBUS / (divisor + 1);

        setupTestFrequencies (currentFBUS);
      }

      #if HARDWARE_CONFIGURATION
      if (F_CPU > 150000000) {
        // Config file has bus frequency set to twice F_BUS
        sumpSetup.clockFrequency *= 2;
      }
      #endif

      sumpSetup.cpuClockTicks = F_CPU / sumpSetup.clockFrequency;

      break;

    case SUMP_CAPTURE_COUNT:
      sumpSetup.samplesToRecord = sumpRX.command[4];
      sumpSetup.samplesToRecord <<= 8;
      sumpSetup.samplesToRecord |= sumpRX.command[3];
      sumpSetup.samplesToRecord <<= 8;
      sumpSetup.samplesToRecord |= sumpRX.command[2];
      sumpSetup.samplesToRecord <<= 8;
      sumpSetup.samplesToRecord |= sumpRX.command[1];
      sumpSetup.samplesToRecord = (sumpSetup.samplesToRecord + 1) * 4;
      break;

    case SUMP_CAPTURE_DELAY:
      sumpSetup.delaySamples = sumpRX.command[4];
      sumpSetup.delaySamples <<= 8;
      sumpSetup.delaySamples |= sumpRX.command[3];
      sumpSetup.delaySamples <<= 8;
      sumpSetup.delaySamples |= sumpRX.command[2];
      sumpSetup.delaySamples <<= 8;
      sumpSetup.delaySamples |= sumpRX.command[1];
      sumpSetup.delaySamples = (sumpSetup.delaySamples + 1) * 4;

      // need this to get OLS client to line up trigger to time 0
      sumpSetup.delaySamples += 2;
      break;
  }
}


void SUMPrecordData(sumpSetupVariableStruct &sumpSetup)
{
  sumpDynamicVariableStruct dynamic;

  set_led_off ();

  sumpSetup.samplesRequested = sumpSetup.samplesToRecord;

  // prevent buffer overruns
  if (sumpSetup.samplesToRecord > (unsigned int) LA_SAMPLE_SIZE * 8) {
    sumpSetup.samplesToRecord = (unsigned int) LA_SAMPLE_SIZE * 8;
    sumpSetup.numberOfChannels = 1;
  } else if (sumpSetup.samplesToRecord > LA_SAMPLE_SIZE * 4) {
    sumpSetup.numberOfChannels = 1;
  } else if (sumpSetup.samplesToRecord > LA_SAMPLE_SIZE * 2) {
    sumpSetup.numberOfChannels = 2;
  } else if (sumpSetup.samplesToRecord > LA_SAMPLE_SIZE) {
    sumpSetup.numberOfChannels = 4;
  } else {
    sumpSetup.numberOfChannels = 8;
  }
  if (sumpSetup.delaySamples > sumpSetup.samplesToRecord) {
    sumpSetup.delaySamples = sumpSetup.samplesToRecord;
  }

#if HARDWARE_CONFIGURATION

  // only 2 SPI channels
  if (sumpSetup.numberOfChannels > 2)
  {
    sumpSetup.numberOfChannels = 2;
  }

  #if Teensy_LC
    if (sumpSetup.cpuClockTicks <= 2)
    {
      // only SPI1 can go up to 24 MHz
      sumpSetup.numberOfChannels = 1;
    }
  #endif

#else

  // if assembly language
  if (sumpSetup.cpuClockTicks <= 8)
  {
    // assembly only does 1 sample per byte
    sumpSetup.numberOfChannels = 8;

    if (sumpSetup.samplesToRecord > LA_SAMPLE_SIZE)
    {
      // set delay to 0 if no trigger, else 50%
      if (sumpSetup.samplesToRecord == sumpSetup.delaySamples)
      {
        sumpSetup.delaySamples = LA_SAMPLE_SIZE;
      }
      else
      {
        sumpSetup.delaySamples = LA_SAMPLE_SIZE / 2;
      }

      sumpSetup.samplesToRecord = LA_SAMPLE_SIZE;
    }
  }

#endif

  // if no trigger level was specified, use first trigger level
  if (sumpSetup.lastTriggerLevel < 0) {
    sumpSetup.lastTriggerLevel = 0;
  }

  // setup
  switch (sumpSetup.numberOfChannels) {
    case 1:
      sumpSetup.samplesPerElement = 8 * 4;
      sumpSetup.sampleMask = 0x01;
      sumpSetup.sampleShift = 1;
      break;
    case 2:
      sumpSetup.samplesPerElement = 4 * 4;
      sumpSetup.sampleMask = 0x03;
      sumpSetup.sampleShift = 2;
      break;
    case 4:
      sumpSetup.samplesPerElement = 2 * 4;
      sumpSetup.sampleMask = 0x0F;
      sumpSetup.sampleShift = 4;
      break;
    default:
      sumpSetup.samplesPerElement = 1 * 4;
      sumpSetup.sampleMask = 0x0FF;
      sumpSetup.sampleShift = 8;
      break;
  }

  // record 4 extra elements, for trigger adjustment later
  sumpSetup.samplesToSend = sumpSetup.samplesToRecord;
  sumpSetup.samplesToRecord = sumpSetup.samplesToRecord + 4 * sumpSetup.samplesPerElement;
  
  sumpSetup.startOfBuffer = (uint32_t *)logicData;
  sumpSetup.endOfBuffer = sumpSetup.startOfBuffer + sumpSetup.samplesToRecord / sumpSetup.samplesPerElement;
  
  if (sumpSetup.triggerMask[0] != 0)
  {
    // number of samples to delay before arming the trigger
    // (if not trigger, then this is 0)
    sumpSetup.delaySamples = sumpSetup.samplesToRecord - sumpSetup.delaySamples;
  }
  else
  {
    // if no trigger mask, it can't delay
    sumpSetup.delaySamples = 0;
  }

  // add one due to truncation
  sumpSetup.delaySizeInElements = (sumpSetup.delaySamples / sumpSetup.samplesPerElement) + 1;

#if HARDWARE_CONFIGURATION
  // for hardware, needs to be an even number
  if (sumpSetup.delaySizeInElements % 2){
    ++sumpSetup.delaySizeInElements;
  }
#endif

  dynamic.triggerSampleIndex = 0;

  // setup timer
  startTimer (sumpSetup.busClockDivisor);

#if HARDWARE_CONFIGURATION

  recordSPIData(sumpSetup, dynamic);

#else

#if Teensy_LC
  if (sumpSetup.cpuClockTicks <= 6)
  {
    sumpStrategy = STRATEGY_ASSEMBLY;
    recordDataAsm6Clocks(sumpSetup, dynamic);
  }
#else
  if (sumpSetup.cpuClockTicks <= 5)
  {
    sumpStrategy = STRATEGY_ASSEMBLY;
    recordDataAsm5Clocks(sumpSetup, dynamic);
  }
#endif
  else if (sumpSetup.cpuClockTicks <= 8)
  {
    sumpStrategy = STRATEGY_ASSEMBLY;
    recordDataAsmWithTrigger(sumpSetup, dynamic);
  }
  else
  {
    sumpStrategy = STRATEGY_NORMAL;

    recordLowSpeedData (sumpSetup, dynamic);
  }

#endif

  if (sumpRunning)
  {
     sendData (sumpSetup, dynamic);
  }

  SUMPreset();
}


inline void waitForTimeout (void)
{
  #ifdef TIMING_DISCRETES     
    digitalWriteFast (TIMING_PIN_0, HIGH);
  #endif

//  while (!timerExpired ());
//  clearTimerFlag ();

   // for speed, to reduce jitter
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

  #ifdef TIMING_DISCRETES     
    digitalWriteFast (TIMING_PIN_0, LOW);
  #endif
}

void blinkled() {
  digitalWriteFast(LED_PIN, HIGH);
  delay(150);
  digitalWriteFast(LED_PIN, LOW);
  delay(100);
}

void blinkledFast() {
  digitalWriteFast(LED_PIN, HIGH);
  delay(10);
  digitalWriteFast(LED_PIN, LOW);
  delay(15);
}

void checkBlinkLED() {
  if (blinkEndTime != 0) {
    if (millis() > blinkEndTime) {
      set_led_off ();
      blinkEndTime = 0;
      blinkStartTime = millis() + 15;
    }
  }
  if (blinkStartTime != 0) {
    if (millis() > blinkStartTime) {
      blinkStartTime = 0;
    }
  }
}

void startBlinkLED() {
  if (blinkStartTime == 0) {
    blinkEndTime = millis() + 150;
    set_led_on ();
  }
}

void startBlinkLEDFast() {
  if (blinkStartTime == 0) {
    blinkEndTime = millis() + 10;
    set_led_on ();
  }
}

inline void set_led_on () {
  digitalWriteFast (LED_PIN, HIGH);
}

inline void set_led_off () {
  digitalWriteFast (LED_PIN, LOW);
}

///////////
//
// PIT Timer routines
//
//////////
inline void clearTimerFlag (void)
{
  TIMER_FLAG_REGISTER = PIT_TFLG_TIF;
}

inline bool timerExpired (void)
{
  return TIMER_FLAG_REGISTER;
}

void startTimer (uint32_t busTicks)
{
  // Enable PIT clock
  SIM_SCGC6 |= SIM_SCGC6_PIT;

  // Enable the PIT module (turn off disable)
  PIT_MCR &= ~PIT_MCR_MDIS;

  // Disable timer
  TIMER_CONTROL_REGISTER = 0;

  // Load value
  TIMER_LOAD_VALUE_REGISTER = busTicks;

  // Enable timer
  TIMER_CONTROL_REGISTER |= PIT_TCTRL_TEN;
}

