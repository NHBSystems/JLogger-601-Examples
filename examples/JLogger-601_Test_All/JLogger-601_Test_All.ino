/*******************************************************************************************
 * This example sketch can be used to test hardware functionality of JLogger-601. 
 * 
 * Jaimy Juliano
 * NHB Sytems
 * October 20th, 2020
 * 
 *******************************************************************************************/


///////////////////////////////////////////////////////////////////////////////////////////////////
// Defines for optional components - COMMENT OUT ANYTHING NOT INSTALLED BEFORE UPLOADING TO LOGGER!
///////////////////////////////////////////////////////////////////////////////////////////////////

//#define HAS_FRAM_CHIP         // TODO: Refactor as specific FRAM chip to avoid confusion
                                // Use this if an FM24CL04B FRAM IC has been installed on
                                // the SOIC pad on the bottom of the board

#define HAS_RFM95_RADIO       // If you uncomment a radio option, you must have the Radiohead library installed.
//#define HAS_RFM69_RADIO
#define RFM_FREQUENCY  915.00   // If testing radio, make sure this frequency is okay for your region

#define HAS_24AA025E64_CHIP     // Included on all boards now

#define TEST_FLASH  
#define TEST_LEDS
#define TEST_SW3V

// Currently the AD7794 and the 2.5V excitation regulator are always tested

///////////////////////////////////  Includes //////////////////////////////////////////////////////

#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
#include "NHB_AD7794.h"


/////////////////////////////// Conditional Includes ///////////////////////////////////////////////
#ifdef TEST_FLASH
  #include <SdFat.h>
  #include <Adafruit_SPIFlash.h>
#endif


// For now I'll just use the RadioHead library to make sure the radio can
// be initialized. Testing with LoRaWAN is too much overhead for a simple
// hardware test. May create a standalone function to test communication 
// with the radio in the future, just to be more self contained.
#ifdef HAS_RFM95_RADIO
  #include <RH_RF95.h>
#endif

#ifdef HAS_RFM69_RADIO
  #include <RH_RF69.h>
#endif
   

#ifdef HAS_24AA025E64_CHIP
  //#include "Microchip24AA02E.h" //Don't need this, can just use extEEPROM library
  #include <extEEPROM.h>
#endif

#ifdef HAS_FRAM_CHIP
  #include <FRAM_MB85RC_I2C.h>
#endif

////////////////////////////// Globals /////////////////////////////////////////////////

const uint32_t AlarmSeconds = 10;

AD7794 adc(PIN_AD7794_CS,4000000, 2.50);    // Create the ADC object

RTCZero rtc;
SPISettings spiSettings;

//mac64 eui;
uint8_t eui[8];

double readings[6]; //6 channels
double adcTemperature;


///////////////////////////// Conditional globals ///////////////////////////////////////

#ifdef HAS_24AA025E64_CHIP
  extEEPROM eep(kbits_2, 1, 16);         //device size, number of devices, page size
#endif

//RFMXX
#ifdef HAS_RFM95_RADIO
  RH_RF95 driver(PIN_RFM_CS,PIN_RFM_IRQ);
  //RHReliableDatagram manager(driver, NODE_ADDRESS); 
#endif

#ifdef HAS_RFM69_RADIO
  RH_RF69 driver(PIN_RFM_CS,PIN_RFM_IRQ);
  //RHReliableDatagram manager(driver, NODE_ADDRESS);
#endif


//FRAM (FM24CL04B right now)
#ifdef HAS_FRAM_CHIP
  const uint8_t framAddress = 0x50;
  const uint16_t framDensity = 4; //4k (4, 16, 64, 128, 256, 512, 1024)
  const uint8_t wpPin = 42; //Don't care, not using
  const bool wpEnabled = false;
  FRAM_MB85RC_I2C framMem(framAddress, wpEnabled, wpPin, framDensity);
#endif

#ifdef TEST_FLASH
  Adafruit_FlashTransport_SPI flashTransport(PIN_FLASH_CS, &SPI);
  Adafruit_SPIFlash flash(&flashTransport);
#endif

////////////////////////////////////////////////////////////////////////////////////////////
// setup() - most of the tests are actually run here
////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  Serial.begin(115200);
  while (!Serial);
  

  //initLoggerPins();
  Wire.begin();
  
  rtc.begin();

  

/////////////////////////// Test LEDs //////////////////////////////////
#ifdef TEST_LEDS
  Serial.println(F("** Testing LEDs **"));
  
  for(int i = 0; i < 10; i++){
    digitalWrite(PIN_BLUE_LED, HIGH);   
    digitalWrite(PIN_LED_RXL, LOW);
    digitalWrite(PIN_EX_EN,HIGH);
    digitalWrite(PIN_LED_TXL, HIGH);
    digitalWrite(PIN_LED_13,HIGH);
    delay(100); // wait a bit
    
    digitalWrite(PIN_BLUE_LED, LOW);   
    digitalWrite(PIN_LED_RXL, HIGH);
    digitalWrite(PIN_EX_EN,LOW);
    digitalWrite(PIN_LED_TXL, LOW);
    digitalWrite(PIN_LED_13,LOW);
    delay(100); // wait a bit
  }
  
  digitalWrite(PIN_BLUE_LED, HIGH); //Turn off blue
  
  Serial.println();
  
#endif


//////////////////////// Test Switched 3.3V (MOSFET) Output //////////////
#ifdef TEST_SW3V
  Serial.println(F("*** Testing Switched 3.3V Supply ***"));
  Serial.println(F("Switched 3.3V pin will turn on for 2 seconds"));
  digitalWrite(PIN_SW_3V, LOW);
  delay(2000);
  digitalWrite(PIN_SW_3V,HIGH);
  Serial.println(F("Turning off"));
  Serial.println();
#endif


////////////////////// Test Radio Inititialization /////////////////////////  
#if defined(HAS_RFM95_RADIO) || defined(HAS_RFM69_RADIO)
    
  Serial.println(F("*** Testing RFMxx Radio ***"));
  
  if (!driver.init()){
    //#ifdef SERIAL_DEBUG
      Serial.println("init failed!");
    //#endif
  }else{
    Serial.println("Radio initialized");
    if (!driver.setFrequency(RFM_FREQUENCY)){
    //#ifdef SERIAL_DEBUG
      Serial.println("setFrequency failed");
    //#endif
    }else{
      Serial.print("Set frequency to ");
      Serial.print(RFM_FREQUENCY);
      Serial.println(" MHz");
    }
  } 

  Serial.println();

  //TODO: Setup RHReliableDatagram and send out a message here
  
#endif


#ifdef HAS_FRAM_CHIP  
  framMem.begin();
  //TODO: Do test
#endif


///////////////////////// Test EUI-64 Chip /////////////////////////////
#ifdef HAS_24AA025E64_CHIP
  Serial.println(F("** Testing 24AA025E64 **"));

  uint8_t eepStatus = eep.begin(eep.twiClock400kHz);   //go fast!
  if (eepStatus) {
    Serial.print(F("extEEPROM.begin() failed, status = "));
    Serial.println(eepStatus);
    //while (1);
  }
  
  //MacReader.readMac64(eui);
  eep.read(0xf8,eui,8);
  Serial.print("EUI64: ");
  for (int i = 0; i < sizeof(eui) / sizeof(eui[0]); i++) {
    Serial.print("0x");
    Serial.print(eui[i],HEX);
    Serial.print(" ");
  }
  Serial.println();

  //Now convert to string
  char devEuiStr[16];
  char buf[3];
  
  for(int i=0;i<8;i++){
    sprintf(buf, "%02x",eui[i]);
    //try some garbage
    devEuiStr[i*2] = buf[0];
    devEuiStr[i*2 + 1] = buf[1];
  }
  devEuiStr[16] = 0; //null terminate
  
  Serial.print("As string: ");
  
  Serial.print(devEuiStr);
  Serial.println();
  Serial.println();

  delay(1000);
#endif

///////////////////////////// Test Flash ////////////////////////////////

#ifdef TEST_FLASH
  //From Adafruit example
  flash.begin();
  
  Serial.println(F("** Testing Flash storage IC **"));
  Serial.print("JEDEC ID: "); Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: "); Serial.println(flash.size());

  uint8_t b1, b2, b3;
  uint32_t JEDEC = flash.getJEDECID();
  //uint16_t ManID = flash.getManID();
  b1 = (JEDEC >> 16);
  b2 = (JEDEC >> 8);
  b3 = (JEDEC >> 0);

  Serial.print(b1,HEX);
  Serial.print(b2,HEX);
  Serial.print(b3,HEX);

  Serial.println();
  Serial.println();

  delay(1000);
#endif

///////////////////////// Test AD7794 //////////////////////////////////

  adc.begin();
  adc.setUpdateRate(16.6);
  
  
  for(int i=0; i < 6; i++){
    adc.setBipolar(i,true);
    //delay(2);
    adc.setGain(i, 128);
    //delay(2);
    adc.setEnabled(i,true);
    //delay(2);
  }

  //Read all 6 channels and spit out on serial
  Serial.println(F("** Test 24 bit ADC Readings ***"));

  Serial.println(F("Turning on excitation voltage"));
  digitalWrite(PIN_EX_EN,LOW); //Turn on excitation

  Serial.println(F("Reading all 6 channels"));
  for(int i=0; i < 6; i++){
    readings[i] = adc.read(i);
    Serial.print("Ch ");
    Serial.print(i);
    Serial.print(": ");    
    Serial.println(readings[i],DEC);    
  }
  //Serial.println();

  //Read the onboard temperature sensor too 
  // (Reading channel 6 returns temperature, but it may need an offset calibration)
  adcTemperature = adc.read(6); 
  Serial.print("And the temperature: ");
  Serial.print(adcTemperature); 
  Serial.println(" C"); 

  Serial.println(F("Turning off excitation"));
  digitalWrite(PIN_EX_EN,HIGH); //Turn off excitation

  Serial.println();


/////////////////////////// Test User Button and Go To sleep ////////////////////////////////


  Serial.println(F("Press user button to go to sleep (USB wil be disconnected)"));
  while (digitalRead(PIN_BUTTON) == HIGH) delay(10);    //wait for button push
  
  
  Serial.println("Going to sleep now, good night");

#if defined(HAS_RFM95_RADIO) || defined(HAS_RFM69_RADIO)      
  driver.sleep();
#endif

  delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////
// loop() - Do nothing except sleep, wakeup, go back to sleep. 
// This is where I test the sleep current draw
/////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  //Serial.print(F("SLEEP DISABLED WHILE WORKING"));
  //while(1){}; //Temporary, will fiddle with sleep later

  //TODO: Maybe I should have it sleep until the button is pushed again?
  
  
  static uint32_t nextAlarm = rtc.getSeconds(); 


  nextAlarm = (nextAlarm + AlarmSeconds) % 60;
  rtc.setAlarmSeconds(nextAlarm);
  rtc.enableAlarm(rtc.MATCH_SS);
  rtc.attachInterrupt(rtcAlarm);
  
  //Serial.println("Going to sleep");
  digitalWrite(PIN_BLUE_LED,HIGH);


  digitalWrite(PIN_LED_TXL, HIGH); //Make sure TX led is off before going to sleep
  
  //rtc.standbyMode();  // <-- Still doesn't handle SysTick properly
  goToSleep();
  
  
  digitalWrite(PIN_BLUE_LED,LOW);
    
  
  delay(2000); 

}


//This may need updating to get USB wakeup working right - not sure?
void goToSleep(){
  
  USBDevice.detach();
  
  //Disable SysTick interupt
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

  //Go to sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  //Code starts here after waking

  //Reenable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
 
  USBDevice.attach();  
  
}

//RTC Alarm Match ISR
void rtcAlarm(){
  //Do nothing for now  
}
