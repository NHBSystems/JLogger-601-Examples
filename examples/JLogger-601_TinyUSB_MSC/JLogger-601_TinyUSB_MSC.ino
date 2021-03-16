/*******************************************************************************************
 * Example for the JLogger-601
 * 
 * This example logs data to a file on the SPI flash IC and uses TinyUSB MSC mode to allow
 * the file system to be mounted like a flash drive when connected to a host. At reset or
 * power on, a new file will be created with an incrementing number appended to the base name. 
 * 
 * The sketch also includes functions to read K Type thermocouples than can be enabled by
 * changing the constant LOG_K_THERM_DEGC to true. You must have 1Meg bias resistors, or
 * thermocouple input modules installed to read thermocouples
 * 
 * If USB is unplugged and a battery is connected, the logger will sleep between readings
 * reducing power consumption to around 13uA.
 * 
 * If USB is re-connected while sleeping, pressing the "user" button will wake the logger
 * and it will show up as a flash drive on the host machine again. If the button is not
 * pressed, it wil still wake at the next reading interval, and will stay awake until 
 * unplugged 
 * 
 * Jaimy Juliano
 * NHB Sytems
 * March, 2021
 * 
 *******************************************************************************************
 *
 * ADDITIONAL NOTES: 
 * 
 * You must select "TinyUSB" under Tools->USB Stack for this sketch to work.
 * 
 * Flash must be formatted ahead of time using the Format_Flash sketch. Loggers sold
 * after 3-1-2021 should already be formatted
 * 
 * When using the TinyUSB stack, serial will not work until after setup() has exited.  
 * Any charecters output before then will be dropped. Additionally using the traditional 
 * while(!Serial) technique in setup() will cause problems. If output is needed at startup
 * we need to do it with some conditional code in loop();
 * 
 * When host is writting, we must not write to the flash from the sketch. This is harder than
 * it seems because even though TinyUSB doesn't run from interupts, the Adafruit flash library 
 * has a nuber of calls to yield() in it. This seems to allow TinyUSB to start (or continue)
 * host transfers right in the middle of the app writing data to flash. The end result is 
 * corrupted files by way of chunks of the host transfer being written into the log file that
 * the sketch is writting. If you want to write a file to the storage from the host, you 
 * should do it shortly after the blue LED goes off, indicating the sketch is done writing to
 * flash until the next reading.
 * 
 * This example uses a simple lock mechanism to try to prevent the sketch and the USB host from
 * writing the flash at the same time. While the sketch is writing logged data, the TinyUSB
 * write callback will return 0, which should tell TinyUSB to try again later. This blocking
 * can cause the write to hang for a pretty long time (~30 sec), but it's better than a bunch 
 * of corrupted files.
 * 
 ********************************************************************************************/


#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
#include "NHB_AD7794.h"

#include <SdFat.h>
#include <Adafruit_SPIFlash.h>
#include "SdFat.h"
#include "Adafruit_TinyUSB.h"

#include <extEEPROM.h> //For 24AA025E64 MAC & EEPROM
#include "avr/dtostrf.h"

////////////////////////////// Globals /////////////////////////////////////////////////

#define BASE_FILE_NAME      "data" //must be <= 6 char long


/* NOTE: You must have bias resistors installed to read thermocouples */
const bool LOG_K_THERM_DEGC = false; // false = Log 6 channels of Voltage.
                                     // true  = Log 6 channels of Type K thermocouple                                     

const uint32_t logPeriodSec = 30;    // Loging interval in seconds

AD7794 adc(PIN_AD7794_CS,4000000, 2.50);    // Create the ADC object

RTCZero rtc;

uint8_t eui[8];

double readings[6]; //6 channels
double adcTemperature;
double busVoltage;

Adafruit_FlashTransport_SPI flashTransport(PIN_FLASH_CS, &SPI);
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;

FatFile root;
char fileName[13] = BASE_FILE_NAME "00.csv";

// USB Mass Storage object
Adafruit_USBD_MSC usb_msc;


volatile bool fsChanged = false;    // not really used right now

volatile bool timeForReading;       // Flag indicating RTC alarm interrupt has fired

volatile bool appFlashLock = false; // To prevent host writes while sketch is writing flash
                                    

extEEPROM eep(kbits_2, 1, 16);      // To access EUI from 24AA025E64
                                    // [device size, number of devices, page size]


//////////////////////////////// Setup ////////////////////////////////////////////

void setup(){

  asm(".global _printf_float"); //Needed for sprintf() float support
  
  analogReadResolution(12);

  //Have to set radio CS high if not using it
  pinMode(PIN_RFM_CS,OUTPUT);
  digitalWrite(PIN_RFM_CS,HIGH); 

  // TODO: May need to put radio to sleep to save power, have to check current draw
  // with radio
  
  rtc.begin();
  
  flash.begin();

  // Init USB Mass storage ////////////
  
  // Set disk vendor id, product id and revision with string up to 8, 16, 4 characters respectively
  usb_msc.setID("NHBSys", "JLogger Flash", "1.0");
  
  // Set callback
  usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
  
  // Set disk size, block size should be 512 regardless of spi flash page size
  usb_msc.setCapacity(flash.size()/512, 512);
  
  // MSC is ready for read/write
  usb_msc.setUnitReady(true);  
  usb_msc.begin();
  

  Serial.begin(115200);
  //while(!Serial) delay(100); // <-- This seems to cause issues, don't do it


  //Get EUI from 24AA025E64 chip
  uint8_t eepStatus = eep.begin(eep.twiClock400kHz);   //go fast!
  if (eepStatus) {
    Serial.print(F("extEEPROM.begin() failed, status = "));
    Serial.println(eepStatus);
    //while (1);
  }
  eep.read(0xf8,eui,8);
  
  
  
  // Init file system on the flash
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount filesystem!");
    Serial.println("Was the flash chip formatted with the fatfs_format example?");
    while(1) delay(10);
  }
  Serial.println("Mounted filesystem!");

  
  
  //Create new filename with incremented number  
  uint8_t baseSize = sizeof(BASE_FILE_NAME) - 1;
  for (uint8_t i = 0; i < 100; i++) {
    fileName[baseSize]    = '0' + i/10;
    fileName[baseSize +1] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! fatfs.exists(fileName)) {
      break;
    }
    yield();
  }

  //Write header info to first line of new data file
  writeHeader(); 
  
  
  adc.begin();
  adc.setUpdateRate(19.6);  
  
  for(int i=0; i < 6; i++){
    adc.setBipolar(i,true);    
    adc.setGain(i, 32);    
    adc.setEnabled(i,true);    
    //yield();
  }
    
  timeForReading = true;  
}


void loop() {
  
  if(timeForReading){

    timeForReading = false;
    
    //Get readings
    digitalWrite(PIN_BLUE_LED,LOW); //Turn on blue LED
    digitalWrite(PIN_EX_EN,LOW);    //Turn on excitation
    delay(10);                      //Give Excitation a bit to stabilize  
    
    //Read the onboard temperature sensor  
    adcTemperature = adc.read(6);   
    
    
    
    //Calc ref voltage for thermocouple cold junction 
    float referenceVoltage = Thermocouple_Ktype_TempToVoltageDegC(adcTemperature); 
   
    //Get readings
    for(int i=0; i < 6; i++){      
      if(LOG_K_THERM_DEGC){        
        readings[i] = Thermocouple_Ktype_VoltageToTempDegC(adc.read(i) + referenceVoltage); 
      }else{
        readings[i] = adc.read(i);
      }   
    }  
    
    digitalWrite(PIN_EX_EN,HIGH); //Turn off excitation      
    
    
    busVoltage = getBusVoltage();
  
    
    writeRecordToFile();
    
  
    //Setup alarm for next sample interval
    static uint32_t nextAlarm = rtc.getSeconds();     
    nextAlarm = (nextAlarm + logPeriodSec) % 60;
    rtc.setAlarmSeconds(nextAlarm);
    rtc.enableAlarm(rtc.MATCH_SS);
    rtc.attachInterrupt(rtcAlarm); 
            
           
  
    if(busVoltage < 4.3){ //If not plugged in, we should sleep
      
      //Let user button wake us up
      attachInterrupt(digitalPinToInterrupt(PIN_BUTTON),buttonWakeup,LOW);

      digitalWrite(PIN_BLUE_LED,HIGH); //Turn off blue LED
      digitalWrite(PIN_LED_TXL, HIGH); //Also make sure TX led is off before going to sleep
         
      yield(); //Give TinyUSB a chance to get caught up before sleeping    
      
      //rtc.standbyMode();  // <-- This still doesn't handle SysTick properly    
      goToSleep();  
    }
    
    else{ //If plugged in, write data out to serial
  
      writeRecordToSerial();  
      
      digitalWrite(PIN_BLUE_LED,HIGH); //Turn off blue LED
      
    }
    
  }
    
}

/////////////////////////////////////////////////////////////////////////////////////

void writeRecordToFile(){
  
  //Don't allow a write if the app is writing to flash.
  //If a write is attempted it will hang for around 30
  //seconds before completing. This isn't ideal, but it's
  //better than corrupted files.
  appFlashLock = true; 
  
                      
  digitalWrite(PIN_LED_TXL, LOW); //just to indicate app writing
  
  File dataFile = fatfs.open(fileName, FILE_WRITE);
  
  // If file opened successfully, write a line to it.  
  if (dataFile) {     

    char writeBuf[128];
    sprintf(writeBuf, "%d,%0.3f,%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
                        rtc.getEpoch(),busVoltage,adcTemperature,
                        readings[0],readings[1],readings[2],
                        readings[3],readings[4],readings[5]);

    dataFile.print(writeBuf);
    
    dataFile.close(); 
    
    digitalWrite(PIN_LED_TXL, HIGH); //just to indicate app done writing

    // clear file system's cache to force refresh
    fatfs.cacheClear();
    
  }  
  appFlashLock = false;  
}

void writeRecordToSerial(){
  //Write out to serial
  Serial.print(busVoltage);
  Serial.print("\t");
  Serial.print(adcTemperature, DEC);
  Serial.print("\t");

  for(int i = 0; i < 6; i++){
    Serial.print(readings[i], DEC);
    Serial.print("\t");
  }  
  Serial.println();
}

void writeHeader(){
  char devEuiStr[17];
  char buf[3];
  
  File dataFile = fatfs.open(fileName, FILE_WRITE);
  
  if (dataFile) { 
    //Put DevEUI at top of file
    dataFile.print(F("Device EUI: "));
        
    for(int i=0;i<8;i++){
      sprintf(buf, "%02x",eui[i]);
      //try some garbage
      devEuiStr[i*2] = buf[0];
      devEuiStr[i*2 + 1] = buf[1];      
    }
    devEuiStr[16] = 0; //null terminate 
  
    dataFile.println(devEuiStr);
    
    //Column labels
    dataFile.println(F("Timestamp,BusV,BoardTemp,CH0,CH1,CH2,CH3,CH4,CH5"));
    dataFile.close();
    delay(10);
  }
}

//Return the bus voltage. This is normally pretty inaccurate due to  
//the high source impedance of the 1Meg voltage divider. Increasing the 
//sampling period really helps, but means it takes significantly longer
//to get the reading. (1M resitors used to keep current down)
float getBusVoltage(){
  
  //Increase the sample period to increase allowable
  //source impedance
  REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(32);
  while (ADC->STATUS.bit.SYNCBUSY == 1);

  //Take the reading
  float val = ((analogRead(PIN_BUS_V) * 3.3) / 4096) *2;

  //Now put it back to reset value
  REG_ADC_SAMPCTRL = ADC_SAMPCTRL_SAMPLEN(0);
  while (ADC->STATUS.bit.SYNCBUSY == 1);

  return val;
}


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
  timeForReading = true;  
}

void buttonWakeup(){
  detachInterrupt(digitalPinToInterrupt(PIN_BUTTON));
}

/*******************************************************************************************/
// Flash USB MSC Callbacks 
// Mostly unchanged from Adafruit TinyUSB example except for lock code in msc_write_cb
/******************************************************************************************/

// Callback invoked when received READ10 command.
// Copy disk's data to buffer (up to bufsize) and 
// return number of copied bytes (must be multiple of block size)
int32_t msc_read_cb (uint32_t lba, void* buffer, uint32_t bufsize)
{
  // Note: SPIFLash Bock API: readBlocks/writeBlocks/syncBlocks
  // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
  return flash.readBlocks(lba, (uint8_t*) buffer, bufsize/512) ? bufsize : -1;
}

// Callback invoked when received WRITE10 command.
// Process data in buffer to disk's storage and 
// return number of written bytes (must be multiple of block size)
//
// NOTE FROM JJ - Returning 0 SHOULD let TinyUSB know the application
// is not ready. 
int32_t msc_write_cb (uint32_t lba, uint8_t* buffer, uint32_t bufsize)
{    
  // Don't allow a write if the app is writing to flash.
  // If a write is attempted it will hang for around 30
  // seconds before completing. This isn't ideal, but it's
  // better than corrupted files.
  if(!appFlashLock){ 
                         
    digitalWrite(LED_BUILTIN, HIGH);       
  
    // Note: SPIFLash Block API: readBlocks/writeBlocks/syncBlocks
    // already include 4K sector caching internally. We don't need to cache it, yahhhh!!
    return flash.writeBlocks(lba, buffer, bufsize/512) ? bufsize : -1;
  }  
  return 0; //Should tell TinyUSB to go away for a bit
}

// Callback invoked when WRITE10 command is completed (status received and accepted by host).
// used to flush any pending cache.
void msc_flush_cb (void)
{
  // sync with flash
  flash.syncBlocks();

  // clear file system's cache to force refresh
  fatfs.cacheClear();

  fsChanged = true;  

  digitalWrite(LED_BUILTIN, LOW);
}


/***************************************************************************************************************
  The following K-type thermocouple functions were borrowed from here:
  https://github.com/annem/AD7193/blob/master/examples/AD7193_Thermocouple_Example/Thermocouple_Functions.ino

****************************************************************************************************************/

float Thermocouple_Ktype_VoltageToTempDegC(float voltage) {
   // http://srdata.nist.gov/its90/type_k/kcoefficients_inverse.html
   float coef_1[] = {0, 2.5173462e1, -1.1662878, -1.0833638, -8.9773540e-1};            // coefficients (in mV) for -200 to 0C, -5.891mv to 0mv
   float coef_2[] = {0, 2.508355e1, 7.860106e-2, -2.503131e-1, 8.315270e-2};            // coefficients (in mV) for 0 to 500C, 0mv to 20.644mv
   float coef_3[] = {-1.318058e2, 4.830222e1, -1.646031, 5.464731e-2, -9.650715e-4};    // whoa, that's hot...
   int i = 5;  // number of coefficients in array
   float temperature;

   float mVoltage = voltage * 1e3;

   if(voltage < 0) {
    temperature = power_series(i, mVoltage, coef_1);
   }else if (voltage > 20.644){
    temperature = power_series(i, mVoltage, coef_3);
   }else{
    temperature = power_series(i, mVoltage, coef_2);
   }

   return(temperature);
}

float Thermocouple_Ktype_TempToVoltageDegC(float temperature) {
  // https://srdata.nist.gov/its90/type_k/kcoefficients.html
  float coef_1[] = {0, 0.3945013e-1, 0.2362237e-4, -0.3285891e-6, -0.4990483e-8};               // coefficients (in mV) for -270 to 0C, -5.891mv to 0mv
  float coef_2[] = {-0.17600414e-1, 0.38921205e-1, 0.1855877e-4, -0.9945759e-7, 0.31840946e-9}; // coefficients (in mV) for 0 to 1372C, 0mv to ....
  float a_coef[] = {0.1185976, -0.1183432e-3, 0.1269686e3};
  int i = 5;  // number of coefficients in array

  float mVoltage;
  float a_power = a_coef[1] * pow((temperature - a_coef[2]), 2);
  float a_results = a_coef[0] * exp(a_power);

  if(temperature < 0) {
    mVoltage = power_series(i, temperature, coef_2) + a_results;
  } else {
    mVoltage = power_series(i, temperature, coef_1);
  }

  return(mVoltage / 1e3);
}

float power_series(int n, float input, float coef[])
 {
      //delay(10);      
      int i;
      float sum=coef[0];
      for(i=1;i<=(n-1);i++)
           sum=sum+(pow(input, (float)i)*coef[i]);
      return(sum);
 }
