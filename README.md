# TeensyLogicAnalyzer
Logic Analyzer for the Teensy development boards (pjrc.com/teensy)

Three modes:

* Basic (default) - I just want to record up to 8 signals at up to about 1 MHz.
* Advanced - I want to read the users guide and get speeds up to 2 MHz on LC and 24 MHz on 3.1/3.2. And more samples when fewer channels.
* Hardware - I only need 1 or 2 channels, but I have a need for speed - LC is 24 MHz, 1 channel or 12 MHz, 2 channels. 3.x is 30 MHz, 2 channels. Most have full triggering capability.

See installation.md for setup.

See users guide for capabilities and how to use.

#### Version 2.0

Adds Hardware configurations. These use 2 SPI channels to read at speeds up to 24 MHz. Also allows full triggering capability at higher speeds. Hardware configuration uses different pins than Basic and Advanced.

#### Version 1.1

Adds Teensy LC Advanced configuration. 1 MHz, 8 MHz sample rates, up to 32k samples at 1 MHz. Also fixed a problem with 3.1 halt at 19.2 MHz.

### Version 1.0

It now compiles under Arduino 1.6.7, adds Teensy LC

This version supports Teensy 3.1/3.2 up to 24 MHz and Teensy LC up to 500 kHz. I don't have a Teensy 3.0 to test with, but it should also support it. If anyone can test it, let me know if it works.

For demonstration purposes, set #define CREATE_TEST_FREQUENCIES 1 in the main file to see 25 kHz signals (varying duty cycles) on channels 4-7 (channels 4-5 on LC).

User Guide is started.