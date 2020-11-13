# JLogger-601 Examples
Repo for examples and demo code.

## Installing The Board Support Package in Arduino
In the Arduino IDE click *File->Preferences->Settings*

![Arduino Menu](/assets/Arduino_Settings_Menu.png)  

![Arduino BM Settings](/assets/Arduino_Settings_BM_URL_All.png)

Paste in the following URL to tell the boards manager where to find json file  

[https://NHBSystems.github.io/nhb_arduino_boards/package_NHBSys_index.json](https://NHBSystems.github.io/nhb_arduino_boards/package_NHBSys_index.json)  

Now you can go into the Arduino Boards Manager, find the JLogger support package, and install it.  

![Arduino BM](/assets/Arduino_BM.png)  
   
Once installed, you can select the board and upload the blink example sketch to check that things are working.  

## Libraries
Additionally you will need to install some libraries to use the onboard hardware, some may depend on how you intend to use the JLogger. In the future I hope to include some of this functionality into an all in one library similar to what Adafruit did for their Circuit Playground boards. Everything except the RadioHead library is available in the Arduino library manager.

### Required
- RTCZero  
- NHB_AD7794
- Adafruit_SPIFlash (will also install Adafruit fork of SDFat)
- extEEPROM

### You'll want one or more of the following for radio support, depending on your needs  

- [RadioHead](https://www.airspayce.com/mikem/arduino/RadioHead/)
- MCCI LMIC LoRaWan Library
- Beelan LoRaWAN 
- *Some other library?* 

Examples are provided for LMIC and Beelan libraries. I hope to add a nice 
example for RadioHead in the future, but I know the standard examples will
work fine as long as you set the pin mappings. The JLogger_601_Test_All example shows the basics off using the onboard hardware and can be used to check that everything is working. It also can be configured to use the RadioHead library to check that an RFM95 (or RFM69) is present and working.
