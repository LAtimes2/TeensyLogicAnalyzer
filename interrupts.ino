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

// Routines to turn off interrupts and turn them back on

bool interrupts_masked = false;

bool getInterruptsMasked (void) {
  return interrupts_masked;
}

void maskInterrupts (void) {

   // Disable SysTick Exception - delay() will not work while disabled
   SYST_CSR &= ~SYST_CSR_TICKINT;

   // Mask USB interrupt
   NVIC_DISABLE_IRQ (IRQ_USBOTG);

   interrupts_masked = true;
}

void unmaskInterrupts (void) {

   // Re-enable SysTick Exception

   // Reading CSR register will also clear COUNTFLAG
   int flag = SYST_CSR;

   // If an interrupt was missed, cause one to occur
   if (flag & SYST_CSR_COUNTFLAG) {
     SCB_ICSR |= SCB_ICSR_PENDSTSET;
   }

   SYST_CSR |= SYST_CSR_TICKINT;

   // Unmask USB interrupt
   NVIC_ENABLE_IRQ (IRQ_USBOTG);

   interrupts_masked = false;
}

// returns true if a USB interrupt is pending (meaning data is available)
inline bool usbInterruptPending (void) {

  return (USB0_ISTAT & ~USB_ISTAT_SOFTOK);
}
