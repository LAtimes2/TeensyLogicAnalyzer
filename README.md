# TeensyLogicAnalyzer
Logic Analyzer for the Teensy development boards (pjrc.com/teensy)

#### Version 1.1

Adds Teensy LC Advanced configuration. 1 MHz, 8 MHz sample rates, up to 32k samples at 1 MHz. Also fixed a problem with 3.1 halt at 19.2 MHz.

### Version 1.0

It now compiles under Arduino 1.6.7, adds Teensy LC

This version supports Teensy 3.1/3.2 up to 24 MHz and Teensy LC up to 500 kHz. I don't have a Teensy 3.0 to test with, but it should also support it. If anyone can test it, let me know if it works.

See installation.md for setup.

For demonstration purposes, set #define CREATE_TEST_FREQUENCIES 1 in the main file to see 25 kHz signals (varying duty cycles) on channels 4-7 (channels 4-5 on LC).

User Guide is started.