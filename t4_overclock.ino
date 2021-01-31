/* Teensy Logic Analyzer
 * Copyright (c) 2021 LAtimes2
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

// This file contains functions for overclocking the Teensy 4. At speeds above 816 MHz, cooling is required.
// Since no cooling is assumed to be used, it needs to limit how long it runs at these speeds.
// To do so, it limits it to the shorter of:
//    1. Time to record data (typically 10's of milliseconds if no trigger is used)
//    2. Temperature interrupt if temperature rises 10 degrees C (or exceeds 70C, whichever is lower)
//    3. Timer interrupt if time exceeds 7 seconds

#include "InternalTemperature.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

const uint32_t OverclockSpeed = 960000000;
const uint32_t OverclockTimeoutUsec = 7 * 1000000;
const uint32_t OverclockMaxTemp = 70;
const uint32_t OverclockMaxTempRise = 10;

float maxMeasuredTemperature;
bool temperatureExceeded = false;
bool overclockTimerExpired = false;
uint32_t originalClockSpeed;
IntervalTimer overclockTimer;

void startOverclocking_720MHz ()
{
  // this speed does not require cooling

  originalClockSpeed = F_CPU;

  // convert clock cycles between samples from original speed to new speed

  // round by adding half of original clock speed
  cpuClockCycles = cpuClockCycles * 720 + ((originalClockSpeed / 2) / 1000000);
  cpuClockCycles /= (originalClockSpeed / 1000000);

  set_arm_clock (720000000);
}

void startOverclocking_816MHz ()
{
  // this speed does not require cooling

  originalClockSpeed = F_CPU;

  // convert clock cycles between samples from original speed to new speed

  // round by adding half of original clock speed
  cpuClockCycles = cpuClockCycles * 816 + ((originalClockSpeed / 2) / 1000000);
  cpuClockCycles /= (originalClockSpeed / 1000000);

  set_arm_clock (816000000);
}

void startOverclocking_960MHz ()
{
  float temperature = InternalTemperature.readTemperatureC ();
  float maxTemp = temperature + OverclockMaxTempRise;

  maxMeasuredTemperature = 0.0;

  if (maxTemp > OverclockMaxTemp)
  {
    maxTemp = OverclockMaxTemp;
  }

  originalClockSpeed = F_CPU;

  overclockTimerExpired = false;
  temperatureExceeded = false;
  stopRecording = false;

  DEBUG_SERIAL(print ("   Pre-temperature: "));
  DEBUG_SERIAL(println (temperature));
  delay (200);

  // if within 2 degrees of the max temperature, don't go into overclocking
  // (caused problems with interrupt going off while still transitioning the clock)
  if (temperature > (OverclockMaxTemp - 2))
  {
    stopRecording = true;
    DEBUG_SERIAL(println ("Temperature too high to overclock, stopping!"));      
  }
  else
  {
    overclockTimer.begin (overclockTimerInterrupt, OverclockTimeoutUsec);
    InternalTemperature.attachHighTempInterruptCelsius (maxTemp, temperatureInterrupt);

    set_arm_clock (OverclockSpeed);
  }
//DEBUG_SERIAL(print("F_CPU oc: "));
//DEBUG_SERIAL(println(F_CPU));
//DEBUG_SERIAL(print("F_BUS oc: "));
//DEBUG_SERIAL(println(F_BUS_ACTUAL));
}

void stopOverclocking (bool fromISR)
{
  maxMeasuredTemperature = InternalTemperature.readTemperatureC ();

  set_arm_clock (originalClockSpeed);

  overclockTimer.end ();
  InternalTemperature.detachHighTempInterrupt ();

digitalWriteFast(LED_PIN, LOW);
  // don't print from within an ISR
  if (!fromISR)
  {
    DEBUG_SERIAL(print ("   Max temperature: "));
    DEBUG_SERIAL(println (maxMeasuredTemperature));

    if (temperatureExceeded)
    {
      DEBUG_SERIAL(println ("Temperature interrupt triggered"));      
    }
    if (overclockTimerExpired)
    {
      DEBUG_SERIAL(println ("Overclock timer interrupt triggered"));      
    }
  }
}

void temperatureInterrupt ()
{
  stopOverclocking (true);
  temperatureExceeded = true;
  stopRecording = true;
}

void overclockTimerInterrupt ()
{
  stopOverclocking (true);
  overclockTimerExpired = true;
  stopRecording = true;
}
