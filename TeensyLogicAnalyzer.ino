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

// Resources used:
//   PIT timer 0
//   Port D pins
//   Serial : interface to SUMP user interface
//   Serial1/2/3 : debug info (if turned on)

//
//  User Configuration settings
//

//#define CREATE_TEST_FREQUENCIES  // if uncommented, it will output frequencies on pins 3 and 6
//#define TIMING_DISCRETES  // if uncommented, set pins for timing

// Debug serial port. Uncomment one of these lines
#define DEBUG_SERIAL(x) 0   // no debug output
//#define DEBUG_SERIAL(x) Serial1.x // debug output to Serial1
//#define DEBUG_SERIAL(x) Serial2.x // debug output to Serial2
//#define DEBUG_SERIAL(x) Serial3.x // debug output to Serial3

//
// Pin definitions (info only - do not change)
//

#define CHAN0 2
#define CHAN1 14
#define CHAN2 7
#define CHAN3 8
#define CHAN4 6
#define CHAN5 20
#define CHAN6 21
#define CHAN7 5

#define LED_PIN 13

#define TIMING_PIN_0 15
#define TIMING_PIN_1 16

//////////////////////////////////////
// End of settings
//////////////////////////////////////

#include <stdint.h>
#include "types.h"

#define VERSION "beta3"

// Teensy 3.0
#if defined(__MK20DX128__)

   // 10k buffer size
   #define LA_SAMPLE_SIZE 12 * 1024

// Teensy 3.1
#elif defined(__MK20DX256__)

   // 58k buffer size
   #define LA_SAMPLE_SIZE 58 * 1024

// Teensy LC
#elif defined(__MKL26Z64__)

   // 58k buffer size
   #define LA_SAMPLE_SIZE 4 * 1024

#else

   // 5k buffer size
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
#define SUMP_TRIG  0xc0
#define SUMP_TRIG_VALS 0xc1
#define SUMP_TRIG_CONFIG 0xc2

// DEBUG_SERIAL 0 causes this warning. It is not a usual warning, so ignoring it is low risk
#pragma GCC diagnostic ignored "-Wunused-value"

// this is the main data storage array. Add 10 extra bytes
// just in case (triggering may use up to 8 extra).
// Needs to be aligned to a 4 byte boundary
byte logicData[LA_SAMPLE_SIZE + 10] __attribute__ ((aligned));

enum strategyType {
  STRATEGY_NORMAL,
  STRATEGY_PACKED,
  STRATEGY_ASSEMBLY,
} sumpStrategy = STRATEGY_NORMAL;

byte sumpNumChannels;
uint32_t sumpDivisor;
uint32_t sumpClockTicks;
uint32_t sumpFrequency;
uint32_t sumpSamples;
uint32_t sumpDelaySamples;
uint32_t sumpRequestedSamples;
uint32_t sumpRunning = 0;
byte sumpTrigMask = 0;
byte sumpTrigValue;

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

// data to set up recording
struct sumpVariableStruct {
  uint32_t bufferSize;
  uint32_t delaySamples = 0;
  uint32_t delaySize = 0;
  uint32_t sampleMask = 0xFF;
  uint32_t sampleShift = 8;
  byte samplesPerByte = 1;
  int samplesToRecord;
  int triggerSampleIndex = 0;
  byte *startOfBuffer;
  byte *endOfBuffer;
  byte *startPtr;
};

void sendData (
  struct sumpSetupVariableStruct sumpSetup,
  struct sumpDynamicVariableStruct dynamic);

void setup()
{
  DEBUG_SERIAL(begin (1000000));  // baud rate of 1 Mbps
  DEBUG_SERIAL(println("Logic Analyzer"));

  // set up pins to record data on
  pinMode(CHAN0, INPUT);
  pinMode(CHAN1, INPUT);
  pinMode(CHAN2, INPUT);
  pinMode(CHAN3, INPUT);
  pinMode(CHAN4, INPUT);
  pinMode(CHAN5, INPUT);
  pinMode(CHAN6, INPUT);
  pinMode(CHAN7, INPUT);

  pinMode(LED_PIN, OUTPUT);

#ifdef TIMING_DISCRETES
  // Pin 0 high = waiting to sample, Pin 1 high = looking for trigger
  pinMode (TIMING_PIN_0, OUTPUT);
  pinMode (TIMING_PIN_1, OUTPUT);

#endif

  blinkled();

#ifdef CREATE_TEST_FREQUENCIES

  /* Use PWM to generate a test signal.
   *  If set on one of the analyzer pins, it will show up when recording.
   *  To test other pins, jumper a PWM output to an analyzer input.
   */

  // PWM available on pins 3-6,9,10,20-23
  // Port D: chan 4(6),5(20),6(21),7(5)

//  analogWriteFrequency (3, 1000000);
//  analogWrite (3, 128);

  analogWriteFrequency (CHAN4, 25000);
  analogWrite (CHAN4, 64);
  analogWrite (CHAN5, 124);

  #if defined(KINETISK)
    analogWrite (CHAN6, 128);
    analogWrite (CHAN7, 192);
  #endif

#endif

  SUMPreset();
}

// main loop
void loop()
{
  byte inByte;

  // loop forever
  while (1) {

    // check for input from SUMP interface
    if (Serial.available() > 0) {
      // blink LED when command is received
      blinkledFast();

      // read and process a byte from serial port
      inByte = Serial.read();

      // write command bytes to debug port
      DEBUG_SERIAL(print(inByte, HEX));
      DEBUG_SERIAL(print(","));

      SUMPprocessCommands(inByte);
    }

    // check for input from Debug port
    if (DEBUG_SERIAL(available()) > 0) {
      blinkledFast();
      SUMPprocessCommands(DEBUG_SERIAL(read()));
    }

    // if commanded to start, then record data
    if (sumpRunning) {
      SUMPrecordData();
    }

    // blink LED every 2 seconds if not recording
    if ((millis() - previousBlinkTime) > 2000) {
      previousBlinkTime = millis();
      blinkled();
    }
  }
}

void SUMPreset(void) {

  sumpRunning = 0;
}

void SUMPprocessCommands(byte inByte) {

  switch (sumpRXstate) { //this is a state machine that grabs the incoming commands one byte at a time

    case C_IDLE:

      processSingleByteCommand (inByte);
      break;

    case C_PARAMETERS:
      sumpRX.parCnt++;
      sumpRX.command[sumpRX.parCnt] = inByte; // store each parameter

      // if all parameters received
      if (sumpRX.parCnt == sumpRX.parameters)
      {
        processFiveByteCommand (sumpRX.command);

        sumpRXstate = C_IDLE;
      }  
      break;

    default:
      blinkled();
      sumpRXstate = C_IDLE;
      break;
  }
}

void processSingleByteCommand (byte inByte){

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
      if (F_CPU == 96000000) {
        Serial.write("Teensy96");
      } else if (F_CPU == 120000000) {
        Serial.write("Teensy120");
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

    // special command for debugging
    case '1':
      DEBUG_SERIAL(write ("sumpSamples: "));
      DEBUG_SERIAL(println (sumpSamples));
      DEBUG_SERIAL(write ("sumpRequestedSamples: "));
      DEBUG_SERIAL(println (sumpRequestedSamples));
      DEBUG_SERIAL(write ("sumpDivisor: "));
      DEBUG_SERIAL(println (sumpDivisor, HEX));
      DEBUG_SERIAL(write ("sumpFrequency: "));
      DEBUG_SERIAL(println (sumpFrequency));
      DEBUG_SERIAL(write ("sumpClockTicks: "));
      DEBUG_SERIAL(println (sumpClockTicks));
      break;

    default: // 5-byte command

      sumpRX.command[0] = inByte; // store first command byte
      sumpRX.parameters = 4; // all long commands are 5 bytes, get 4 parameters
      sumpRX.parCnt = 0; // reset the parameter counter
      sumpRXstate = C_PARAMETERS;
      break;
  }
}

void processFiveByteCommand (byte command[])
{
  uint32_t divisor;

  switch (sumpRX.command[0]) {

    case SUMP_TRIG: // mask for bits to trigger on
      sumpTrigMask = sumpRX.command[1];
      break;

    case SUMP_TRIG_VALS: // value to trigger on
      sumpTrigValue = sumpRX.command[1];
      break;

    case SUMP_CNT:
      sumpSamples = sumpRX.command[2];
      sumpSamples <<= 8;
      sumpSamples |= sumpRX.command[1];
      sumpSamples = (sumpSamples + 1) * 4;

      sumpDelaySamples = sumpRX.command[4];
      sumpDelaySamples <<= 8;
      sumpDelaySamples |= sumpRX.command[3];
      sumpDelaySamples = (sumpDelaySamples + 1) * 4;

      // need this to get OLS client to line up trigger to time 0
      sumpDelaySamples += 2;

      sumpRequestedSamples = sumpSamples;

      // prevent buffer overruns
      if (sumpSamples > (unsigned int) LA_SAMPLE_SIZE * 8) {
        sumpSamples = (unsigned int) LA_SAMPLE_SIZE * 8;
        sumpNumChannels = 1;
      } else if (sumpSamples > LA_SAMPLE_SIZE * 4) {
        sumpNumChannels = 1;
      } else if (sumpSamples > LA_SAMPLE_SIZE * 2) {
        sumpNumChannels = 2;
      } else if (sumpSamples > LA_SAMPLE_SIZE) {
        sumpNumChannels = 4;
      } else {
        sumpNumChannels = 8;
      }
      if (sumpDelaySamples > sumpSamples) {
        sumpDelaySamples = sumpSamples;
      }
      break;

    case SUMP_DIV:
      divisor = 0;
      divisor = sumpRX.command[3];
      divisor <<= 8;
      divisor |= sumpRX.command[2];
      divisor <<= 8;
      divisor |= sumpRX.command[1];

      // by setting device.dividerClockspeed = F_BUS, divisor is exactly equal to value to put in timer
      sumpDivisor = divisor;

      sumpFrequency = F_BUS / (divisor + 1);
      sumpClockTicks = F_CPU / sumpFrequency;

      break;
  }
}


void SUMPrecordData(void) {

  sumpVariableStruct sv;

  byte samplesPerByte = 1;
  byte sampleMask;
  byte sampleShift;
  
  byte *startOfBuffer = logicData;

  byte *endOfBuffer;
  byte *startPtr;

  // setup
    switch (sumpNumChannels) {
      case 1:
        samplesPerByte = 8;
        sampleMask = 0x01;
        sampleShift = 1;
        break;
      case 2:
        samplesPerByte = 4;
        sampleMask = 0x03;
        sampleShift = 2;
        break;
      case 4:
        samplesPerByte = 2;
        sampleMask = 0x0F;
        sampleShift = 4;
        break;
      default:
        samplesPerByte = 1;
        sampleMask = 0x0FF;
        sampleShift = 8;
        break;
    }

  sv.bufferSize = sumpSamples / samplesPerByte;

int samplesPerElement = samplesPerByte * 4;

  sv.samplesToRecord = sumpSamples + 2 * samplesPerElement;
  
  endOfBuffer = startOfBuffer + sv.samplesToRecord / samplesPerByte;
  
  sv.delaySamples = sumpSamples - sumpDelaySamples;
  sv.delaySize = sv.delaySamples / samplesPerByte;
  
  // set pointer to beginning of data buffer
  startPtr = startOfBuffer;

  sv.samplesPerByte = samplesPerByte;
  sv.startPtr = startPtr;
  sv.endOfBuffer = endOfBuffer;
  sv.startOfBuffer = startOfBuffer;
  sv.sampleMask = sampleMask;
  sv.sampleShift = sampleShift;
  sv.triggerSampleIndex = 0;

  // setup timer
  startTimer (sumpDivisor);

  if (sumpClockTicks <= 8)
  {
    sumpStrategy = STRATEGY_ASSEMBLY;
  }
  else
  {
    sumpStrategy = STRATEGY_NORMAL;

    recordLowSpeedData (sv);
//recordByteData (sv);
  }
  

  sumpSetupVariableStruct sumpSetup;
  sumpDynamicVariableStruct dynamic;

  sumpSetup.delaySamples = sv.delaySamples;
  sumpSetup.sampleMask = sv.sampleMask;
  sumpSetup.sampleShift = sv.sampleShift;
  sumpSetup.samplesPerElement = sv.samplesPerByte * 4;
  sumpSetup.samplesRequested = sumpRequestedSamples;
  sumpSetup.samplesToRecord = sv.samplesToRecord;
  sumpSetup.samplesToSend = sv.bufferSize;
  sumpSetup.startOfBuffer = (uint32_t *)sv.startOfBuffer;
  sumpSetup.endOfBuffer = (uint32_t *)sv.endOfBuffer;

  dynamic.triggerSampleIndex = sv.triggerSampleIndex;

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

