## To install:

1. Download files from github. Download zip. Unzip to any directory - desktop or documents will work.
2. Get a sump GUI. I use Logic Sniffer (ols.lxtreme.nl). Download the latest zip file and unzip to any directory. Requires Java JRE to be installed.
3. If using Logic Sniffer, copy the configuration files from TeensyLogicAnalyzer/LogicSniffer to ols-0.9.7.2/plugins

## To compile:

* Teensy 3.1/3.2 - set Tools -> Clock Speed to 96 MHz optimized (overclock)
* Teensy 3.0 - set Tools -> Clock Speed to 96 MHz (overclock)
* Teensy LC - set Tools -> Clock Speed to 48 MHz optimized

It will get a warning, "Low memory available, stability problems may occur". This is ok - we want to record as much data as possible :)

If you don't have any signals to connect, you can change the $define CREATE_TEST_FREQUENCIES to 1 to generate test signals. This will output PWN signals on certain pins. No jumpering of pins is necessary to see the data.

## To run:

1. Load Teensy with the logic analyzer code and run (LED should be blinking every 2 seconds).
2. In ols-0.9.7.2 directory, run or click on run.bat.
3. Select Capture -> Begin Capture
4. Set Connection type to Serial port, Analyzer port to your Teensy COM port (verify at Teensyduino Tools -> Port), Port Speed 115200bps (may not matter since USB)
5. Set Device type as follows: (if it's not available, did you copy the .cfg file correctly?)
  *      Teensy 3.0/3.1/3.2 - Teensy 96 MHz OLS
  *      Teensy LC - Teensy LC 48 MHz OLS
6. Select Acquisition tab and set Sampling Rate and Recording Size to desired settings.
7. If a trigger is desired, select Triggers tab, set Enabled, set Before/After ratio, and select Mask and Value.
8. Select Capture. Data should show up in window.

Channel |  Pin
:-------:|:---:
   0    |  2
   1    | 14
   2    |  7
   3    |  8
   4    |  6
   5    | 20
   6    | 21
   7    |  5

See Users_Guide.pdf for more details and how to build/use Advanced configurations.
