To install:

1. Download files from github. Download zip. Unzip to any directory - desktop or documents will work.
2. Get a sump GUI. I use Logic Sniffer (ols.lxtreme.nl). Download the latest zip file and unzip to any directory. Requires Java JRE to be installed.
3. If using Logic Sniffer, copy the configuration files from TeensyLogicAnalyzer/LogicSniffer to ols-0.9.7.2/plugins

To run:

1. Load Teensy with the logic analyzer code and run (LED should be blinking every 2 seconds).
2. In ols-0.9.7.2 directory, run or click on run.bat.
3. Select Capture -> Begin Capture
4. Set Connection type to Serial port, Analyzer port to your Teensy COM port, Port Speed 115200bps (may not matter since USB)
5. Set Device type to Teensy 96 MHz OLS. If it's not available, did you copy the .cfg file correctly?
6. To test, click Show device metadata. Device type should be Teensy96.
7. Select Acquisition tab and set Sampling Rate and Recording Size to desired settings. (any size above 58 kB will reduce the number of channels captured).
8. If a trigger is desired, select Triggers tab, set Enabled, set Before/After ratio, and select Mask and Value.
9. Select Capture. Data should show up in window.

