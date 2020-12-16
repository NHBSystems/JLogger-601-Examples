/*************************************************************************************
 * JLogger 601 example using Beelan-LoRaWAN library with AD7794 and low power 
 * sleep (~14 uA)
 * 
 * Sends readings (battery voltage and 6 ADC channels + temperature from the
 * AD7794) over LoRaWAN every 60 seconds, sleeping in between. 
 * 
 * Includes an example of downlink command that changes the Tx/sleep interval
 * 
 * The required Javascript uplink Decode and downlink Encode functions are  
 * included in comment at the end of the sketch. You will need to copy and 
 * paste functions into Chirpstack codec section (or TTN, whatever).
 * 
 * Written for and tested with Chirpstack, should work on TTN (or some other 
 * network) with minimal changes to the javascript function names and signature
 * 
 * Tested with: 
 *  1.3.0 (Modified - SPI transaction mod)
 *  1.5.1
 * 
 ************************************************************************************/



#include <lorawan.h>
#include <SPI.h>
#include <Wire.h>
#include <RTCZero.h>
#include "NHB_AD7794.h"
#include <extEEPROM.h>


//#define SERIAL_DEBUG

#define MIN_INTERVAL        10     //Minimum sleep/transmit interval
#define MAX_INTERVAL        3600   //Maximum sleep/transmit interval
#define DEFAULT_INTERVAL    30     //Interval we start at

#define EUI64_I2C_ADDR      0x50   //Not currently used, but left here for reference
#define EUI64_MAC_ADDR      0xF8
#define EUI64_MAC_LENGTH    0x08

#define DLCMD_SET_RATE      1
//#define DLCMD_OTHERCMD      2


//// OTAA credentials /////////////////////////////
char devEui[16];  //WE will fill this with EUI from 24AA025E64 chip
/**************** YOU NEED TO SET THESE! *********/
const char *appEui = "0000000000000000"; //Chirpstack doesnt use this, but need to change for TTN
const char *appKey = "00000000000000000000000000000000"; //<- You need to change this


//Tx/sleep timing
uint16_t sleepInterval = DEFAULT_INTERVAL; 
uint8_t nextAlarmSec;
uint8_t nextAlarmMin;


//Pin mapping for JLogger 601
const sRFM_pins RFM_pins = {
  .CS = PIN_RFM_CS, //2,
  .RST = PIN_RFM_RST, //27,
  .DIO0 = PIN_RFM_DIO_0, //7,
  .DIO1 = PIN_RFM_DIO_1, //38,
  .DIO2 = 255, //Not connected
  .DIO5 = 255, //Not connected
};


AD7794 adc(PIN_AD7794_CS,4000000, 2.50);    // Create the ADC object

RTCZero rtc;

extEEPROM eep(kbits_2, 1, 16);  //device size, number of devices, page size

uint8_t eui[8];


/////////////////////////// Setup //////////////////////////////////////////////

void setup() {
  asm(".global _printf_float"); // Enable sprintf float support
  
  //delay(2000);

  #ifdef SERIAL_DEBUG
    while (! Serial);
    Serial.begin(115200);
  #endif  

//  initLoggerPins();

  analogReadResolution(12);

  //Initialize the 24AA025E64 
  uint8_t eepStatus = eep.begin(eep.twiClock400kHz);   //go fast!
  if (eepStatus) {
    #ifdef SERIAL_DEBUG
      Serial.print(F("extEEPROM.begin() failed, status = "));
      Serial.println(eepStatus);
    //while (1);
    #endif
  }

  //Read the EUI
  eep.read(EUI64_MAC_ADDR, eui, EUI64_MAC_LENGTH); 

  //...And convert it to a HEX string
  char buf[3];
  for(int i=0;i<8;i++){
    sprintf(buf, "%02x",eui[i]);   
    devEui[i*2] = buf[0];
    devEui[i*2 + 1] = buf[1];
  }
  devEui[16] = 0; //NULL Termination

  #ifdef SERIAL_DEBUG
    Serial.print(F("devEUI: "));
    Serial.println(devEui);
  #endif


  //Initialize RTC
  rtc.begin();

  //Set time actual time here if needed

  nextAlarmSec = rtc.getSeconds();
  nextAlarmMin = rtc.getMinutes();


  // Initialize the AD7794
  // Setup for bridge type sensors (e.g. load cell or pressure gauge)
  // Set update rate to 19.6, 90 dB noise rejection at 60 Hz
  // Set bipolar mode
  // Set set all channels to a gain of 128
  adc.begin();
  adc.setUpdateRate(16.6); //< Slow, but 16.6Hz rate enables 50/60hz noise rejection
  for(int i=0; i < 6; i++){
    adc.setBipolar(i,true);
    //delay(2);
    adc.setGain(i, 128);
    //delay(2);
    adc.setEnabled(i,true);
    //delay(2);
  }



  //Initialize radio and LoRaWAN
  if(!lora.init()){
    #ifdef SERIAL_DEBUG
      Serial.println("RFM95 not detected");
    #endif
    delay(1000);
    while(1){
      //Just freak out
      digitalWrite(LED_BUILTIN,LOW);
      delay(500);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(1000);
    }    
  }

  // Set LoRaWAN Class change CLASS_A or CLASS_C
  lora.setDeviceClass(CLASS_A);

  // Set Data Rate
  lora.setDataRate(SF7BW125);

  // set channel to random
  lora.setChannel(MULTI);
  
  // Put OTAA Key and DevAddress here
  lora.setDevEUI(devEui);
  lora.setAppEUI(appEui);
  lora.setAppKey(appKey);

  //Join procedure
  bool isJoined;
  do {
    #ifdef SERIAL_DEBUG
      Serial.println("Joining...");
    #endif 
    isJoined = lora.join();
    
    //wait for 10s to try again
    delay(5000);
  }while(!isJoined);
  
  #ifdef SERIAL_DEBUG
    Serial.println("Joined to network");
  #endif

}

/////////////////////////// Loop //////////////////////////////////////////

void loop() {  
  
  char sendBuffer[32];
  char recvBuffer[255];
  float readings[8];  
  uint8_t recvStatus = 0;



  // Take readings ////////////////////////////////

  digitalWrite(PIN_EX_EN,LOW); //Turn on excitation
  
  //Get the bus/battery voltage
  readings[0] = getBusVoltage();

  //Read AD7794 chip temperature
  readings[1] = adc.read(6);

  //Read 6 ADC channels
  for(int i=0; i < 6; i++){
    readings[i+2] = adc.read(i);
  }

  digitalWrite(PIN_EX_EN,HIGH); //Turn off excitation
  

  // Send out the readings ///////////////////////
  
  //Pack it up...
  memcpy(sendBuffer, readings, sizeof(readings)); 

  //...and ship it out  
  lora.sendUplink(sendBuffer, sizeof(sendBuffer), 0,1);  //This function has changed to include the mport arg
                                                         //in the 1.5+ Version of the library. 
                                                         
  //lora.sendUplink(sendBuffer, sizeof(sendBuffer), 0);  //Earlier library versions
  
  // Check Lora RX
  lora.update();



  // Get reply and parse /////////////////////

  recvStatus = lora.readData(recvBuffer);    
     
  if(recvStatus) {
    parseDownlink(recvBuffer);
    
    //Blink LED to show we got a downlink
    digitalWrite(PIN_BLUE_LED,LOW);
    delay(500);
    digitalWrite(PIN_BLUE_LED,HIGH);
  }


  // Timing & Sleep Code //////////////////////

  nextAlarmSec = (nextAlarmSec + (sleepInterval % 60)) % 60;
  nextAlarmMin = (nextAlarmMin + (sleepInterval / 60)) % 60;

  #ifdef SERIAL_DEBUG    
    Serial.print("nextAlarmSec: ");
    Serial.println(nextAlarmSec);
    Serial.print("nextAlarmMin: ");
    Serial.println(nextAlarmMin);
  #endif  

  if(sleepInterval <= 60){
    rtc.setAlarmSeconds(nextAlarmSec);    
    rtc.enableAlarm(rtc.MATCH_SS); 
  }else{
    rtc.setAlarmSeconds(nextAlarmSec);  
    rtc.setAlarmMinutes(nextAlarmMin);
    rtc.enableAlarm(rtc.MATCH_MMSS);
  }
  
  rtc.attachInterrupt(rtcAlarm);

  
  digitalWrite(LED_BUILTIN,LOW); 

  #ifdef SERIAL_DEBUG
    Serial.println(F("Going to (Fake) sleep now"));
    delay(sleepInterval * 1000);
    Serial.println(F("Good morning!"));
  #else
    RFM_Sleep(); // Beelan library doesn't put the radio to sleep so do it manually

    // RTCZero library still does not handle SysTick properly.
    // Posted here:https://community.atmel.com/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
    // On 1-8-2019
    
    //rtc.standbyMode(); // <-- Doesn't handle SysTick properly 
    
    goToSleep(); // Safe sleep function. Could probably also use ArduinoLowPower library
    
  #endif
  
  digitalWrite(LED_BUILTIN,HIGH); //Waste of power, but this is just an example 
    
}

////////////////////////// Functions /////////////////////////////////////////

// Parse Any Downlink Messages, the BeeLan library doesn't seem to acknowledge 
// ports so I am using a simple command byte scheme where the first byte in the
// payload is a the "command". 
// This is just to demonstrate the concept
void parseDownlink(char * buf){
  
  //Struct to represent simple message
  struct dlMessage_t{
    uint8_t   cmd;
    uint8_t   payload[32]; //Need to look into actual appropriate max payload size
  } message;

  memset(message.payload,0,sizeof(message.payload));  
  memcpy(&message, buf, sizeof(buf));

  //Debug info
  #ifdef SERIAL_DEBUG
    Serial.print(F("Message received [ "));
    Serial.print(F("cmd: "));
    Serial.print(message.cmd,HEX);
    Serial.print(F(" Payload: "));
    for(int i=0;i<sizeof(message.payload);i++){
      Serial.print(message.payload[i],HEX);
      Serial.print(' ');
    }
    Serial.println(" ]");
  #endif


  // We only have one command for now, but more can be added
  // easily with additional if() statements.

  // Set the transmit interval
  if(message.cmd == DLCMD_SET_RATE){
    
    //We got it! 
    digitalWrite(PIN_BLUE_LED,LOW);
    delay(1000);
    digitalWrite(PIN_BLUE_LED,HIGH);
    
    uint16_t arg1; 
    arg1 =  message.payload[1] | message.payload[0] << 8 ;   
    

    #ifdef SERIAL_DEBUG
      Serial.print("arg1 = ");
      Serial.println(arg1,BIN);
    #endif
 
    //TODO: Set sleep interval here 
    if((arg1 >= MIN_INTERVAL) && (arg1 < MAX_INTERVAL)){
      sleepInterval = arg1;

      //Just re-sync these whenever we change the interval to avoid timing issues
      nextAlarmSec = rtc.getSeconds();
      nextAlarmMin = rtc.getMinutes();
      
      #ifdef SERIAL_DEBUG
        Serial.print(F("Set inteval to "));
        Serial.println(arg1);
      #endif
    }
    
  }else{
    //Unknown command
    #ifdef SERIAL_DEBUG
      Serial.println(F("Unknown command"));
    #endif
    
  }
  
}


// The Beelan-LoRaWAN libray doesn't have a method to put the radio to sleep
// so we'll do it manually
void RFM_Sleep(){

    //Borrowed from the RadioHead library
    const uint8_t RH_RF95_MODE_SLEEP      = 0x00;
    const uint8_t RH_RF95_REG_01_OP_MODE  = 0x01;
    const uint8_t RH_SPI_WRITE_MASK       = 0x80;
    
    //Set NSS pin Low to start communication
    digitalWrite(PIN_RFM_CS,LOW);

    //Send Addres with MSB 1 to make it a write command
    SPI.transfer(RH_RF95_REG_01_OP_MODE | RH_SPI_WRITE_MASK);
    //Send Data
    SPI.transfer(RH_RF95_MODE_SLEEP);

    //Set NSS pin High to end communication
    digitalWrite(PIN_RFM_CS,HIGH);
}


//Return the bus voltage. This would normally be pretty inaccurate due to  
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

//Sleep function that handles the SysTick interrupt properly.
//Issue Posted here: https://community.atmel.com/comment/2625116#comment-2625116
//On 1-8-2019
//This may need updating to get USB wakeup working right - not sure?
void goToSleep(){
  
  bool restoreUSBDevice = false;

  //Try to handle attached USB gracefully
	if (SERIAL_PORT_USBVIRTUAL) {
		USBDevice.standby();
	} else {
		USBDevice.detach();
		restoreUSBDevice = true;
	}

  //Disable SysTick interupt
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

  //Go to sleep
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  //Code starts here after waking

  //Reenable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
 
  if (restoreUSBDevice) {
		USBDevice.attach();
	}
  
}

// RTC Alarm Match ISR
void rtcAlarm(){
  //Do nothing for now
  
}

/***********************************************************************************
Example Javascript Decode/Encode functions for Chirpstack. This will also work for
TTN but you have to change the Decode signature slightly to Decoder(bytes, port)
************************************************************************************

// Helper that converts incoming raw byte back to floats 
// From https://www.thethingsnetwork.org/forum/t/decode-float-sent-by-lopy-as-node/8757/2
// Based on https://stackoverflow.com/a/37471538 by Ilya Bursov
function bytesToFloat(bytes) {
  // JavaScript bitwise operators yield a 32 bits integer, not a float.
  // Assume LSB (least significant byte first).
  var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
  var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
  var e = bits>>>23 & 0xff;
  var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
  var f = sign * m * Math.pow(2, e - 150);
  return f;
  
  // The “bias” value, which is documented to be 127 for 32 bits single-precision 
  // IEEE-754 floating point, has been replaced by 150 in m * Math.pow(2, e - 150). 
  // This is effectively m × 2-23 × 2e-127 a.k.a. (m / 223) × 2e-127, and is a nice 
  // optimization to get the 24th implicit leading bit in the “mantissa”.
}  



////  Decode for Battery, temparature and 6 AIN channels from AD7794 //////////////////////////

// !!! Use Decoder(bytes, port) for The Things Network
function Decode(fPort, bytes){
  var payloadObj = {"BattVoltage":"",
                    "Temperature":"",
                    "Ch_0":"",
                    "Ch_1":"",
                    "Ch_2":"",
                    "Ch_3":"",
                    "Ch_4":"",
                    "Ch_5":""};
  
  //var floats = new Float32Array(2);
  var b = 0;
  payloadObj.BattVoltage = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Temperature = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_0 = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_1 = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_2 = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_3 = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_4 = bytesToFloat(bytes.slice(b, b+=4));
  payloadObj.Ch_5 = bytesToFloat(bytes.slice(b, b+=4));
  
  return payloadObj;
}



//// Javascript Encode function for sending a 1 byte command with a single uint16 arg ////////

function Encode(fPort, obj) {
  var bytes = [];
  
  bytes[0] = obj.command & 0xFF;
  
  bytes[1] = (obj.arg1 >> 8) & 0xFF;
  bytes[2] = obj.arg1 & 0xFF;
  

  return bytes; 
}

*****************************************************************************************/
