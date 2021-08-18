# JLogger-601 Examples
This is the repo for example code for using the JLogger-601 LoRa data logger. There are currently 3 examples. 

*JLogger-601_Test_All* - This is example shows how to intitialize and check the onboard hardware. I use it to check boards after assembly, but it's also usefull to show how to get things up and running.  

*JLogger-601_LoraWan_Example-BeelanLib* - This is example shows how to use the JLogger-601 with the Beelan-LoRaWan library. It's a nice little library for a simplified application that just needs to get data to the server and maybe receive some downlinks. It does not however support MAC commands (yet?). This example also shows how to put JLogger to sleep between readings, and processing downlink commands.

*JLogger-601_LoRaWan_Example-LMIC* - This is example shows how to use the JLogger-601 with the MCCI Arduino-LMIC library. It is tested with version 3.2.0, which is fully LoRaWan compliant. It is based on the clasic LMIC example that you may have seen before, but includes (safely) sleeping and processing downlinks, among other things.

*JLogger-601_TinyUSB_MSC* - Example of how to log to the onboard flash, and use the TinyUSB library to allow the flash to be mounted like a USB flash drive to retrieve the logged data.

A detailed guide for the examples can be found [here](https://nhbsystems.com/jlogger-601-getting-started/#Example_Sketches)  

If you have not already done so, you will need to install the board support package and required libraries before trying to run these examples.

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
work fine as long as you set the pin mappings. The JLogger_601_Test_All example shows the basics of using the onboard hardware and can be used to check that everything is working. It also can be configured to use the RadioHead library to check that an RFM95 (or RFM69) is present and working.
