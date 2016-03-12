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

// Teensy 3.0/3.1/3.2
#if defined(KINETISK)

//
// Teensy 3.1 MCU has a partial implementation of SPI1 in the hardware - there is no clock signal available.
// Luckily, the logic analyzer doesn't need the clock signal, only the MISO signal, so it can use SPI1.
//

#define KINETISK_SPI1 (*(KINETISK_SPI_t *)0x4002D000)
#define SPI1_MCR      (KINETISK_SPI1.MCR)   // DSPI Module Configuration Register
#define SPI1_TCR      (KINETISK_SPI1.TCR)   // DSPI Transfer Count Register
#define SPI1_CTAR0    (KINETISK_SPI1.CTAR0) // DSPI Clock and Transfer Attributes Register, In Master Mode
#define SPI1_SR       (KINETISK_SPI1.SR)    // DSPI Status Register
#define SPI1_PUSHR    (KINETISK_SPI1.PUSHR) // DSPI PUSH TX FIFO Register In Master Mode
#define SPI1_POPR     (KINETISK_SPI1.POPR)  // DSPI POP RX FIFO Register

#endif

// Forward declarations
void rearrangeBufferValues (sumpSetupVariableStruct &sv,
                            sumpDynamicVariableStruct &dynamic,
                            bool adjustChan1Left);
inline void startSPIClock (bool multipleChannels,
                           uint32_t cpuClockTicks);


//
//  Include record_hadware.h multiple times with different defines
//

// single channel
#include "record_hardware.h"

// single channel with trigger
#define USE_TRIGGER 1
#include "record_hardware.h"
#undef USE_TRIGGER

// single channel with pre-trigger
#define USE_PRE_TRIGGER 1
#include "record_hardware.h"
#undef USE_PRE_TRIGGER

// multiple channels
#define MULTIPLE_CHANNELS 1
#include "record_hardware.h"

// multiple channels with trigger
#define USE_TRIGGER 1
#include "record_hardware.h"
#undef USE_TRIGGER

// multiple channels with pre-trigger
#define USE_PRE_TRIGGER 1
#include "record_hardware.h"
#undef USE_PRE_TRIGGER

#undef MULTIPLE_CHANNELS


void recordSPIData (sumpSetupVariableStruct &sv,
                    sumpDynamicVariableStruct &dynamic)
{
  bool adjustChan1Left = false;
  bool multipleChannels = (sv.numberOfChannels > 1);

  if (multipleChannels)
  {
    if (sv.triggerMask)
    {
#if Teensy_LC
      // 12 MHz - can't support full triggering
      if (sv.cpuClockTicks <= 4)
      {
        recordSPIData_MultiChannel_Pretrigger (sv, dynamic);
      }
      else
#endif
      {
        recordSPIData_MultiChannel_Trigger (sv, dynamic);
      }
    }
    else
    {
      recordSPIData_MultiChannel (sv, dynamic);
    }
  }
  else
  {
    if (sv.triggerMask)
    {
#if Teensy_LC
      // 24 MHz - can't support full triggering
      if (sv.cpuClockTicks <= 2)
      {
        recordSPIData_SingleChannel_Pretrigger (sv, dynamic);
      }
      else
#endif
      {
        recordSPIData_SingleChannel_Trigger (sv, dynamic);
      }
    }
    else
    {
      recordSPIData_SingleChannel (sv, dynamic);
    }
  }

  if (multipleChannels)
  {
    // offset between channels is most likely to occur at higher speeds. An
    // attempt is made to compensate in startSPIClock and this then
    // removes that compensation
    if (sv.cpuClockTicks <= 16)
    {
      adjustChan1Left = true;
    }

    // adjust data from how SPI stores it to how send_data expects it
    rearrangeBufferValues (sv, dynamic, adjustChan1Left);
  }
}

void rearrangeBufferValues (sumpSetupVariableStruct &sv,
                            sumpDynamicVariableStruct &dynamic,
                            bool adjustChan1Left)
{
  if (sv.numberOfChannels > 1)
  {
    uint16_t *inputPtr = (uint16_t *)sv.startOfBuffer;
    uint32_t *outputPtr = (uint32_t *)sv.startOfBuffer;
    uint16_t chan0Values;
    uint16_t chan1Values;
    uint16_t firstMSB;
    uint32_t newValues;
    uint16_t mask;

    firstMSB = *(inputPtr + 1) >> 15;
    
    while (outputPtr < sv.endOfBuffer)
    {
      // fix the data to alternate each sample
      chan0Values = *inputPtr;
      ++inputPtr;
      chan1Values = *inputPtr;
      ++inputPtr;

      newValues = 0;
      mask = 0x8000;

      for (int index = 15; index >= 0; --index)
      {
        newValues += ((chan0Values & mask) >> index) << (index * 2);

        // if recorded at same time
        if (!adjustChan1Left)
        {
          newValues += ((chan1Values & mask) >> index) << (index * 2 + 1);
        }
        else
        {
          // if not first bit
          if (index > 0)
          {
            // shift channel 1 to the left 1 bit
            newValues += (((chan1Values << 1) & mask) >> index ) << (index * 2 + 1);
          }
          else
          {
            // if not last value in buffer
            if (outputPtr < (sv.endOfBuffer - 1))
            {
              // LSB is MSB of next value
              chan1Values = *(inputPtr + 1);
              newValues += ((chan1Values & 0x8000) >> 15) << 1;
            }
            else // last value in buffer - use MSB of first value
            {
              newValues += firstMSB << 1;
            }
          }
        }

        mask = mask >> 1;
      }

      *outputPtr = newValues;
      ++outputPtr;
DEBUG_SERIAL(print("ip: "));
DEBUG_SERIAL(print((int)inputPtr, HEX));
DEBUG_SERIAL(print(" op: "));
DEBUG_SERIAL(print((int)outputPtr, HEX));
DEBUG_SERIAL(print(" ch0: "));
DEBUG_SERIAL(print((int)chan0Values, HEX));
DEBUG_SERIAL(print(" ch1: "));
DEBUG_SERIAL(print((int)chan1Values, HEX));
DEBUG_SERIAL(println(""));
//delay(1);
    }
  }
}


#if Teensy_LC

// SPI register values
//                     SPI master, system enable, clock phase (to get continuous clock)
const uint8_t SPI_C1 = SPI_C1_MSTR | SPI_C1_SPE | SPI_C1_CPHA;
//                     16 bit mode
const uint8_t SPI_C2 = SPI_C2_SPIMODE;
//                     FIFO mode
const uint8_t SPI_C3 = SPI_C3_FIFOMODE;


void spi0Initialize()
{
  SIM_SCGC4 |= SIM_SCGC4_SPI0;
  SPI0_C1 = SPI_C1;
  SPI0_C2 = SPI_C2;

  if (1) {
    CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // MOSI0 = 11 (PTC6)
  } else {
    CORE_PIN7_CONFIG = PORT_PCR_MUX(2); // MOSI0 = 7 (PTD2)
  }
  if (CHAN1 == 12) {
    CORE_PIN12_CONFIG = PORT_PCR_MUX(2);  // MISO0 = 12 (PTC7)
  } else {
    CORE_PIN8_CONFIG = PORT_PCR_MUX(2);  // MISO0 = 8 (PTD3)
  }
  if (0) {
    CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // SCK0 = 13 (PTC5)
  } else {
    CORE_PIN14_CONFIG = PORT_PCR_MUX(2); // SCK0 = 14 (PTD1)
  }
}

void spi1Initialize()
{
  SIM_SCGC4 |= SIM_SCGC4_SPI1;
  SPI1_C1 = SPI_C1;
  SPI1_C2 = SPI_C2;

  if (0) {
    CORE_PIN0_CONFIG = PORT_PCR_MUX(2);  // MOSI1 = 0  (PTB16)
  } else {
    CORE_PIN21_CONFIG = PORT_PCR_MUX(2); // MOSI1 = 21 (PTD6)
  }
  if (CHAN0 == 1) {
    CORE_PIN1_CONFIG = PORT_PCR_MUX(2);  // MISO1 = 1  (PTB17)
  } else {
    CORE_PIN5_CONFIG = PORT_PCR_MUX(2);  // MISO1 = 5  (PTD7)
  }
  CORE_PIN20_CONFIG = PORT_PCR_MUX(2); // SCK1 = 20 (PTD5)

}

static uint32_t spiGetClockSetting (uint32_t clock, uint32_t baseFrequency) {

  uint32_t br;

  if        (clock >= baseFrequency /   2) { br = SPI_BR_SPPR(0) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /   4) { br = SPI_BR_SPPR(1) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /   6) { br = SPI_BR_SPPR(2) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /   8) { br = SPI_BR_SPPR(3) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /  10) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /  12) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /  14) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /  16) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(0);
  } else if (clock >= baseFrequency /  20) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(1);
  } else if (clock >= baseFrequency /  24) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(1);
  } else if (clock >= baseFrequency /  28) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(1);
  } else if (clock >= baseFrequency /  32) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(1);
  } else if (clock >= baseFrequency /  40) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(2);
  } else if (clock >= baseFrequency /  48) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(2);
  } else if (clock >= baseFrequency /  56) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(2);
  } else if (clock >= baseFrequency /  64) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(2);
  } else if (clock >= baseFrequency /  80) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(3);
  } else if (clock >= baseFrequency /  96) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(3);
  } else if (clock >= baseFrequency / 112) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(3);
  } else if (clock >= baseFrequency / 128) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(3);
  } else if (clock >= baseFrequency / 160) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(4);
  } else if (clock >= baseFrequency / 192) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(4);
  } else if (clock >= baseFrequency / 224) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(4);
  } else if (clock >= baseFrequency / 256) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(4);
  } else if (clock >= baseFrequency / 320) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(5);
  } else if (clock >= baseFrequency / 384) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(5);
  } else if (clock >= baseFrequency / 448) { br = SPI_BR_SPPR(6) | SPI_BR_SPR(5);
  } else if (clock >= baseFrequency / 512) { br = SPI_BR_SPPR(7) | SPI_BR_SPR(5);
  } else if (clock >= baseFrequency / 640) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(6);
  } else if (clock >= baseFrequency / 768) { br = SPI_BR_SPPR(5) | SPI_BR_SPR(6);
  } else if (clock >= baseFrequency /1280) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(7);
  } else if (clock >= baseFrequency /2560) { br = SPI_BR_SPPR(4) | SPI_BR_SPR(8);
  } else      /* baseFrequency / 4096 */   { br = SPI_BR_SPPR(7) | SPI_BR_SPR(8);
  }

  return br;
}

void spiDisable (bool multipleChannels)
{
  SPI1_C1 = 0;

  if (multipleChannels)
  {
    SPI0_C1 = 0;
  }
}

void spi0Setup(uint32_t clock)
{
  // set baud rate
  SPI0_BR = spiGetClockSetting (clock, F_BUS);

  // read status register to clear flags
  SPI0_S;
}

void spi1Setup(uint32_t clock)
{
  // set baud rate
  SPI1_BR = spiGetClockSetting (clock, F_PLL/2);

  // read status register to clear flags
  SPI1_S;
}

inline void spi0StartTransfer ()
{
  SPI0_DL = 0;
  SPI0_DH = 0;
}

inline void spi1StartTransfer ()
{
  SPI1_DL = 0;
  SPI1_DH = 0;
}

inline void startSPIClock (bool multipleChannels,
                           uint32_t cpuClockTicks)
{
  volatile uint8_t *SPI0_DH_reg = &SPI0_DH;
  volatile uint8_t *SPI1_DH_reg = &SPI1_DH;
  uint32_t SPI_DH_value = 0;

//digitalWriteFast (TIMING_PIN_0, LOW);
  if (multipleChannels)
  {
    // start channel 1 (SPI0) before channel 0 (SPI1) since channel 0 is
    // used to tell if done (0 will finish first if it is started first)
    SPI1_DL = 0;
    SPI0_DL = 0;

    if (cpuClockTicks <= 4)
    {
      // start the SPI channels 4 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "strb %[SPI_DH_value], [%[SPI0_DH_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "strb %[SPI_DH_value], [%[SPI1_DH_reg]]\n\t"
                    : [SPI0_DH_reg] "+r" (SPI0_DH_reg),
                      [SPI1_DH_reg] "+r" (SPI1_DH_reg),
                      [SPI_DH_value] "+r" (SPI_DH_value)
                                       :: "cc");
    }
    else if (cpuClockTicks <= 8)
    {
      // start the SPI channels 8 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "strb %[SPI_DH_value], [%[SPI0_DH_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "strb %[SPI_DH_value], [%[SPI1_DH_reg]]\n\t"
                    : [SPI0_DH_reg] "+r" (SPI0_DH_reg),
                      [SPI1_DH_reg] "+r" (SPI1_DH_reg),
                      [SPI_DH_value] "+r" (SPI_DH_value)
                                       :: "cc");
    }
    else if (cpuClockTicks <= 16)
    {
      // start the SPI channels 16 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "strb %[SPI_DH_value], [%[SPI0_DH_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "strb %[SPI_DH_value], [%[SPI1_DH_reg]]\n\t"
                    : [SPI0_DH_reg] "+r" (SPI0_DH_reg),
                      [SPI1_DH_reg] "+r" (SPI1_DH_reg),
                      [SPI_DH_value] "+r" (SPI_DH_value)
                                       :: "cc");
    }
    else
    {
      SPI0_DH = 0;
      SPI1_DH = 0;      
    }
  }
  else
  {
    spi1StartTransfer ();
  }

//digitalWriteFast (TIMING_PIN_0, HIGH);
  // wait for transmit buffer to be ready for next transfer
  while (!(SPI1_S & SPI_S_SPTEF)) ; // wait
//digitalWriteFast (TIMING_PIN_0, LOW);
// No FIFO: SPTEF, FIFO: not needed. don't start next transfer yet

  // start a 2nd transfer while waiting for first to complete
  spi1StartTransfer ();

  if (multipleChannels)
  {
    // clear status so more data can be sent
    SPI0_S;

    // start a 2nd transfer while waiting for first to complete
    spi0StartTransfer ();
  }
}


#else // 3.1

void spi0Initialize()
{
// TEMPORARY
#if F_CPU == 24000000
  // config divisors: 24 MHz core, 8 MHz bus, 24 MHz flash, USB = 96 / 2
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(3) | SIM_CLKDIV1_OUTDIV2(15) |  SIM_CLKDIV1_OUTDIV4(3);
#endif
#if F_CPU == 48000000
  // config divisors: 24 MHz core, 8 MHz bus, 24 MHz flash, USB = 96 / 2
  SIM_CLKDIV1 = SIM_CLKDIV1_OUTDIV1(1) | SIM_CLKDIV1_OUTDIV2(3) |  SIM_CLKDIV1_OUTDIV4(3);
#endif
   SIM_SCGC6 |= SIM_SCGC6_SPI0 + SIM_SCGC6_SPI1;


   
////   SIM_SCGC6 |= SIM_SCGC6_SPI0;
//   SPI0_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);
//   SPI0_CTAR0 = SPI_CTAR_FMSZ(7) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
//   SPI0_CTAR1 = SPI_CTAR_FMSZ(15) | SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);

CORE_PIN11_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // DOUT/MOSI = 11 (PTC6)

   // MISO can be pin 12 or pin 8
   if (CHAN1 == 12) {
     CORE_PIN12_CONFIG = PORT_PCR_MUX(2);  // DIN/MISO = 12 (PTC7)
   } else {
     CORE_PIN8_CONFIG = PORT_PCR_MUX(2);  // DIN/MISO = 8 (PTD3)
   }

//   CORE_PIN13_CONFIG = PORT_PCR_DSE | PORT_PCR_MUX(2); // SCK = 13 (PTC5)
   CORE_PIN14_CONFIG = PORT_PCR_MUX(2); // SCK = 14 (PTD1)

//CORE_PIN5_CONFIG = PORT_PCR_MUX(3);  // UART0_TX = 64 ()

   
//   SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F);
}

void spi1Initialize()
{
   SIM_SCGC6 |= SIM_SCGC6_SPI0 + SIM_SCGC6_SPI1;
////   SIM_SCGC6 |= SIM_SCGC6_SPI1;

   // MISO can be pin 1 or pin 26
   if (CHAN0 == 1) {
     CORE_PIN1_CONFIG = PORT_PCR_MUX(2);  // DIN/MISO = 40 ()

     // move UART TX to pin 5
     CORE_PIN5_CONFIG = PORT_PCR_MUX(3);  // UART0_TX = 64 ()

   } else {
     CORE_PIN26_CONFIG = PORT_PCR_MUX(2);  // DIN/MISO = 2 ()
   }

CORE_PIN0_CONFIG = PORT_PCR_MUX(2);  // MOSI

}

void spi0Setup(uint32_t clock)
{
  SPI0_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

  // set control register for correct clock setting and 16 bit transfers
  SPI0_CTAR0 = spiGetClockSetting (clock) | SPI_CTAR_FMSZ(15); //// | SPI_CTAR_LSBFE;

  // turn off HALT, set continuous clock, clear both FIFOs
//  SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

  // clear Rx FIFO not empty flag, Tx FIFO Underflow, RxFIFO Overflow
  SPI0_SR = SPI_SR_RFDF | SPI_SR_TFUF | SPI_SR_RFOF;
}

void spi1Setup(uint32_t clock)
{
  SPI1_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

  // set control register for correct clock setting and 16 bit transfers
  SPI1_CTAR0 = spiGetClockSetting (clock) | SPI_CTAR_FMSZ(15); //// | SPI_CTAR_LSBFE;

//  SPI1_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;

  // clear Rx FIFO not empty flag, Tx FIFO Underflow, RxFIFO Overflow
  SPI1_SR = SPI_SR_RFDF | SPI_SR_TFUF | SPI_SR_RFOF;
}

static uint32_t spiGetClockSetting (uint32_t clock) {

  uint32_t t;

  if        (clock >= F_BUS /   2) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /   3) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /   4) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /   5) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_DBR | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /   6) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /   8) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
  } else if (clock >= F_BUS /  10) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(0) | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /  12) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(1);
  } else if (clock >= F_BUS /  16) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
  } else if (clock >= F_BUS /  20) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(1) | SPI_CTAR_CSSCK(0);
  } else if (clock >= F_BUS /  24) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
  } else if (clock >= F_BUS /  32) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3);
  } else if (clock >= F_BUS /  40) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
  } else if (clock >= F_BUS /  48) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3);
  } else if (clock >= F_BUS /  56) { t = SPI_CTAR_PBR(3) | SPI_CTAR_BR(3) | SPI_CTAR_CSSCK(2);
  } else if (clock >= F_BUS /  64) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
  } else if (clock >= F_BUS /  80) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(4) | SPI_CTAR_CSSCK(3);
  } else if (clock >= F_BUS /  96) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
  } else if (clock >= F_BUS / 128) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
  } else if (clock >= F_BUS / 160) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(5) | SPI_CTAR_CSSCK(4);
  } else if (clock >= F_BUS / 192) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
  } else if (clock >= F_BUS / 256) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
  } else if (clock >= F_BUS / 320) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(6) | SPI_CTAR_CSSCK(5);
  } else if (clock >= F_BUS / 384) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
  } else if (clock >= F_BUS / 512) { t = SPI_CTAR_PBR(0) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
  } else if (clock >= F_BUS / 640) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(7) | SPI_CTAR_CSSCK(6);
  } else if (clock >= F_BUS / 768) { t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(7);
  } else if (clock >= F_BUS /1280) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(8) | SPI_CTAR_CSSCK(6);
  } else if (clock >= F_BUS /2560) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(9) | SPI_CTAR_CSSCK(8);
  } else if (clock >= F_BUS /5120) { t = SPI_CTAR_PBR(2) | SPI_CTAR_BR(10)| SPI_CTAR_CSSCK(9);
  } else {   /* F_BUS / 10240 */     t = SPI_CTAR_PBR(1) | SPI_CTAR_BR(11)| SPI_CTAR_CSSCK(10);
  }

  return t;
}

void spiDisable (bool multipleChannels)
{
  // wait for data to finish transmitting
  delay (10);

  SPI1_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);

  if (multipleChannels)
  {
    SPI0_MCR = SPI_MCR_MDIS | SPI_MCR_HALT | SPI_MCR_PCSIS(0x1F);
  }


delay (10);
SIM_SCGC6 &= ~(SIM_SCGC6_SPI0 + SIM_SCGC6_SPI1);


}

inline void startSPIClock (bool multipleChannels,
                           uint32_t cpuClockTicks)
{
SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
SPI1_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
  // ensure that FIFO's are empty (had a problem with this)
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;

  if (multipleChannels)
  {
    SPI0_POPR;
    SPI0_POPR;
    SPI0_POPR;
    SPI0_POPR;
    SPI0_POPR;

    // start channel 1 (SPI0) before channel 0 (SPI1) since channel 0 is
    // used to tell if done (0 will finish first if it is started first)

    volatile uint32_t *SPI0_PUSHR_reg = &SPI0_PUSHR;
    volatile uint32_t *SPI1_PUSHR_reg = &SPI1_PUSHR;
    uint32_t SPI_PUSHR_value = SPI_PUSHR_CONT;

    volatile uint32_t *SPI0_MCR_reg = &SPI0_MCR;
    volatile uint32_t *SPI1_MCR_reg = &SPI1_MCR;
    uint32_t SPI_MCR_value = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE;

    if (cpuClockTicks <= 4)
    {
      // start the SPI channels 4 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "str %[SPI_MCR_value], [%[SPI0_MCR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_MCR_value], [%[SPI1_MCR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"

                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"
                    : [SPI0_PUSHR_reg] "+r" (SPI0_PUSHR_reg),
                      [SPI1_PUSHR_reg] "+r" (SPI1_PUSHR_reg),
                      [SPI_PUSHR_value] "+r" (SPI_PUSHR_value),
                      [SPI0_MCR_reg] "+r" (SPI0_MCR_reg),
                      [SPI1_MCR_reg] "+r" (SPI1_MCR_reg),
                      [SPI_MCR_value] "+r" (SPI_MCR_value)
                                       :: "cc");
    }
    else if (cpuClockTicks <= 8)
    {
      // start the SPI channels 8 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "str %[SPI_MCR_value], [%[SPI0_MCR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_MCR_value], [%[SPI1_MCR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"

                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"
                    : [SPI0_PUSHR_reg] "+r" (SPI0_PUSHR_reg),
                      [SPI1_PUSHR_reg] "+r" (SPI1_PUSHR_reg),
                      [SPI_PUSHR_value] "+r" (SPI_PUSHR_value),
                      [SPI0_MCR_reg] "+r" (SPI0_MCR_reg),
                      [SPI1_MCR_reg] "+r" (SPI1_MCR_reg),
                      [SPI_MCR_value] "+r" (SPI_MCR_value)
                                       :: "cc");
    }
    else if (cpuClockTicks <= 12)
    {
      // start the SPI channels 12 ticks apart, so they are exactly
      // 1 sample apart. This will be corrected in rearrangeBufferValues
      // after recording is complete.
      asm volatile (".align 2\n\t"
                    "str %[SPI_MCR_value], [%[SPI0_MCR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_MCR_value], [%[SPI1_MCR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"

                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"
                    : [SPI0_PUSHR_reg] "+r" (SPI0_PUSHR_reg),
                      [SPI1_PUSHR_reg] "+r" (SPI1_PUSHR_reg),
                      [SPI_PUSHR_value] "+r" (SPI_PUSHR_value),
                      [SPI0_MCR_reg] "+r" (SPI0_MCR_reg),
                      [SPI1_MCR_reg] "+r" (SPI1_MCR_reg),
                      [SPI_MCR_value] "+r" (SPI_MCR_value)
                                       :: "cc");
    }
    else
    {
//SPI0_MCR |= SPI_MCR_CONT_SCKE;
//SPI1_MCR |= SPI_MCR_CONT_SCKE;
SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
SPI1_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CLR_TXF | SPI_MCR_CLR_RXF;
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;
  SPI1_POPR;
  SPI0_POPR;
  SPI0_POPR;
  SPI0_POPR;
  SPI0_POPR;
  SPI0_POPR;
SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE;
SPI1_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE;
      SPI0_PUSHR = SPI_PUSHR_CONT;
    SPI0_PUSHR = SPI_PUSHR_CONT;
      SPI1_PUSHR = SPI_PUSHR_CONT;
    SPI1_PUSHR = SPI_PUSHR_CONT;
    }


/*
      asm volatile (".align 2\n\t"
//                    "nop\n\t"
//                    "nop\n\t"
                    "str %[SPI_MCR_value], [%[SPI0_MCR_reg]]\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
                    "nop\n\t"
//                    "nop\n\t"

//                    "nop\n\t"

                    "str %[SPI_MCR_value], [%[SPI1_MCR_reg]]\n\t"
//                    "nop\n\t"
//                    "nop\n\t"
//                    "nop\n\t"
//                    "nop\n\t"

                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
//                    "nop\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI0_PUSHR_reg]]\n\t"
                    "str %[SPI_PUSHR_value], [%[SPI1_PUSHR_reg]]\n\t"
                    : [SPI0_PUSHR_reg] "+r" (SPI0_PUSHR_reg),
                      [SPI1_PUSHR_reg] "+r" (SPI1_PUSHR_reg),
                      [SPI_PUSHR_value] "+r" (SPI_PUSHR_value)
,[SPI0_MCR_reg] "+r" (SPI0_MCR_reg),
                 [SPI1_MCR_reg] "+r" (SPI1_MCR_reg),
                 [SPI_MCR_value] "+r" (SPI_MCR_value)
                                       :: "cc");
*/

    // continue to fill FIFO's
    SPI0_PUSHR = SPI_PUSHR_CONT;
    SPI0_PUSHR = SPI_PUSHR_CONT;
  }
  else
  {
SPI1_MCR = SPI_MCR_MSTR | SPI_MCR_PCSIS(0x1F) | SPI_MCR_CONT_SCKE;

    // start
    SPI1_PUSHR = SPI_PUSHR_CONT;
  SPI1_PUSHR = SPI_PUSHR_CONT;
  }

  // continue to fill FIFO's
  SPI1_PUSHR = SPI_PUSHR_CONT;
  SPI1_PUSHR = SPI_PUSHR_CONT;
}

#endif
