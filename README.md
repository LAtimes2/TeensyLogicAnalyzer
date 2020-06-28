# TeensyLogicAnalyzer
Logic Analyzer for the Teensy development boards (pjrc.com/teensy)

(4/22/20) NEW: First beta release for TEENSY 4.0. Only supports 3 channels. Next release will be 8 channels.\
(6/28/20)      Added TEENSY 4.1 beta release. Still only 3 channels. Official release is coming soon.

Two modes:

* Basic (default) - I just want to record up to 8 signals at up to about 1 MHz. Higher speeds (6 to 48 MHz) are available with some restrictions.
* Hardware - I only need 1 or 2 channels, but I have a need for speed - LC is 24 MHz, 3.2 is 72 MHz, 3.5 is 60 MHz, 3.6 is 120 MHz. Most have full triggering capability.

See users guide for capabilities and how to use.

If you are using the Logic Analyzer and have a github account, please leave a comment at https://github.com/LAtimes2/TeensyLogicAnalyzer/issues/1 with your Teensy and OLS versions that you use.

# New updated OLS GUI is available for Windows and Linux

Now you can get started on Windows or Linux in 3 easy steps:

1a. Windows - Download OLS .zip (https://github.com/LAtimes2/ols/releases/latest), unzip, and run run.bat

1b. Linux - Download OLS .tar.gz (https://github.com/LAtimes2/ols/releases/latest), extract, and run run.sh

2. Select Capture -> Begin Capture, and set Analyzer port to COM port or /dev/tty for the Teensy

3. Select Device type for your Teensy type, then select Load Firmware

When done, select Capture and you can start looking at data. For a demo on Teensy without external signals, select Teensy Demo when loading firmware and channels 4-8 will have PWM data on them (be sure not to connect anything to the Teensy since these channels are outputs in Demo mode).

#### Version 4.0

New Run-Length Encoding (RLE) support. This only records data when it changes, so can record up to 100 times longer, depending on how often the data changes. To select it, enable Run Length Encoding on Acquisition tab.
Added pre-built hex files for Teensy 3.0.

#### Version 3.2

Add support for Teensy 3.5, 3.6. Add new commands for sample sizes >256k.

#### Version 3.1

No new functions. Preparation for new OLS GUI. Better support for sigrok. Eliminated Advanced configuration - Basic now supports it all.

#### Version 3.0

Adds Complex Triggers (up to 4 stages). Allows edge triggers or things like 4th clock select pulse. For 3.x, increased maximum hardware speed to 72 MHz.

#### Version 2.0

Adds Hardware configurations. These use 2 SPI channels to read at speeds up to 24 MHz. Also allows full triggering capability at higher speeds. Hardware configuration uses different pins than Basic and Advanced.

#### Version 1.1

Adds Teensy LC Advanced configuration. 1 MHz, 8 MHz sample rates, up to 32k samples at 1 MHz. Also fixed a problem with 3.1 halt at 19.2 MHz.

### Version 1.0

It now compiles under Arduino 1.6.7, adds Teensy LC

This version supports Teensy 3.1/3.2 up to 24 MHz and Teensy LC up to 500 kHz. I don't have a Teensy 3.0 to test with, but it should also support it. If anyone can test it, let me know if it works.

For demonstration purposes, set #define CREATE_TEST_FREQUENCIES 1 in the main file to see 25 kHz signals (varying duty cycles) on channels 4-7 (channels 4-5 on LC).

User Guide is started.
