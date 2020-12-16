/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * 
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * -------------------------------------------------------------------------------
 * 
 * Copyright (c) 2020 Jaimy Juliano NHB Systems
 * 
 * This classic LMIC example has been modified for use with the JLogger-601 board. 
 * 
 * Noteworthy additions include:
 *  -Set the DEVEUI to the EUI read from the onboard 24AA025E64 chip
 *  -Read and transmit data from the onboard AD7794 24 bit ADC
 *  -Put the board into low power (~14uA) sleep state between RX/TX cycles
 *  -Basic example check if it's safe to sleep, or if LMIC has tasks to complete
 *  -Updates millis() to reflect time spent sleeping (IMPORTANT)
 *  -Show example of 2 ways of processing downlink commands (using a command byte,
 *   and also using the port)
 * 
 * Right now this has only been tested with Chirpstack server. It should work just
 * fine with TTN too, I just haven't tried it lately. (an older version worked fine)
 * You should only have to uncomment USE_TNN and comment out USE_CHIPSTACK below.
 * 
 * Of course, as mentioned above, you have to fill in your APPEUI and APPKEY info too
 * 
 * The required Javascript uplink Decode and downlink Encode functions are  
 * included in comment at the end of the sketch. You will need to copy and 
 * paste functions into Chirpstack codec section (or TTN, whatever).
 * 
 * Tested with MCCI Arduino-LMIC Ver. 3.2.0 on 11-10-2020
 **********************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <Wire.h>
#include <RTCZero.h>
#include "NHB_AD7794.h"
#include <extEEPROM.h>

#define SERIAL_DEBUG
#define dbgSerial Serial1

#define MCCI_LMIC                  //Comment out if using a classic LMiC such as
                                   //https://github.com/matthijskooijman/arduino-lmic)

#define MIN_INTERVAL        10     //Minimum sleep/transmit interval
#define MAX_INTERVAL        3600   //Maximum sleep/transmit interval
#define DEFAULT_INTERVAL    30     //Interval we start at

#define EUI64_I2C_ADDR      0x50   //Not currently used, but left here for reference
#define EUI64_MAC_ADDR      0xF8
#define EUI64_MAC_LENGTH    0x08

#define DLCMD_SET_RATE      1
//#define DLCMD_USERCMD1      2
//#define DLCMD_USERCMD2      3
// ...

//#define USE_TTN
#define USE_CHIRPSTACK

#ifdef USE_TTN
  // This EUI must be LSB (Least-Significant-Byte) first.
  static const u1_t PROGMEM APPEUI[8]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
  
  /*** USE THIS IF YOU WANT TO USE YOUR OWN EUI ***/
  // This should also be LSB (Least-Significant-Byte) first. 
  // static const u1_t PROGMEM DEVEUI[8]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  // void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

  uint8_t devEUI[8]; ////Will fill this with EUI from 24AA025E64 chip in setup
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, devEUI, 8);}
  
  // This key should be MSB (Most-Significant-Byte) first
  static const u1_t PROGMEM APPKEY[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
#endif

#ifdef USE_CHIRPSTACK
  static const u1_t PROGMEM APPEUI[8]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //Chirpstack doesn't use an APPEUI
  void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

  /*** USE THIS IF YOU WANT TO USE YOUR OWN EUI ***/
  // This should also be LSB (Least-Significant-Byte) first. 
  // static const u1_t PROGMEM DEVEUI[8]= {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  // void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
   
  uint8_t devEUI[8]; ////Will fill this with EUI from 24AA025E64 chip in setup
  void os_getDevEui (u1_t* buf) { memcpy_P(buf, devEUI, 8);}
  static const u1_t PROGMEM APPKEY[16] = {0x93,0xaa,0x30,0xad,0x90,0x36,0x4f,0xa8,0x18,0xd3,0x11,0xe7,0xd8,0x8e,0x9a,0xef}; //Chirpstack testing appkey
  //static const u1_t PROGMEM APPKEY[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
  void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}
#endif

static osjob_t sendjob;


uint8_t sendBuffer[32];
float readings[8];


uint16_t sleepInterval = DEFAULT_INTERVAL;
uint8_t nextAlarmSec;
uint8_t nextAlarmMin;


//Pin mapping for JLogger-601 
const lmic_pinmap lmic_pins = {
    .nss  = PIN_RFM_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst  = PIN_RFM_RST, 
    .dio  = {PIN_RFM_DIO_0, PIN_RFM_DIO_1, LMIC_UNUSED_PIN},
    
    #ifdef MCCI_LMIC
    .rxtx_rx_active = 0,
    .rssi_cal = 8,         // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
    #endif
};


AD7794 adc(PIN_AD7794_CS,4000000, 2.50);    // Create the ADC object

RTCZero rtc;

extEEPROM eep(kbits_2, 1, 16);  //device size, number of devices, page size

uint8_t eui[8];



/////////////////////////// Setup //////////////////////////////////////////////

void setup() {
    delay(2000);

    #ifdef SERIAL_DEBUG
      //while (! dbgSerial);    
      dbgSerial.begin(115200);
      dbgSerial.println(F("Starting"));
    #endif
        
    analogReadResolution(12);



    //Initialize the 24AA025E64 
    uint8_t eepStatus = eep.begin(eep.twiClock400kHz);  
    if (eepStatus) {
      #ifdef SERIAL_DEBUG
        dbgSerial.print(F("extEEPROM.begin() failed, status = "));
        dbgSerial.println(eepStatus);
      //while (1);
      #endif
    }

    //Read the EUI (will reverse for LSB below)
    eep.read(EUI64_MAC_ADDR, devEUI, EUI64_MAC_LENGTH); 

    #ifdef SERIAL_DEBUG
      // Convert it to a HEX string
      char buf[3];
      char devEuiStr[16];
      for(int i=0;i<8;i++){
        sprintf(buf, "%02x",devEUI[i]);   
        devEuiStr[i*2] = buf[0];
        devEuiStr[i*2 + 1] = buf[1];
      }
      devEuiStr[16] = 0; //NULL Termination
    
      dbgSerial.print(F("devEUI: "));
      dbgSerial.println(devEuiStr);
    #endif

    reverseBytes(devEUI,EUI64_MAC_LENGTH);


    //Initialize RTC
    rtc.begin();
  
    /** Set time actual time here if needed **/
  
    nextAlarmSec = rtc.getSeconds();
    nextAlarmMin = rtc.getMinutes();

    
    //Initialize the AD7794
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

    //Initialize LMIC
    os_init();
    
    //Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    
    //May be needed, not sure?
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    
    LMIC_setAdrMode(0);   
    
    LMIC_selectSubBand(1); // Count starts at zero, so this is actually subband 2 (a bit annoying)
        
    //TTN apparently uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;
  
    //Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}


////////////////////////////// Loop //////////////////////////////////////////

void loop() {  
  //Everything happens in event handler below  
  os_runloop_once();
}


////////////////////////// Main event handler /////////////////////////////////

void onEvent (ev_t ev) {
    #ifdef SERIAL_DEBUG
      dbgSerial.print(os_getTime());
      dbgSerial.print(": ");
    #endif
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_SCAN_TIMEOUT"));
            #endif
            break;
        case EV_BEACON_FOUND:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_BEACON_FOUND"));
            #endif
            break;
        case EV_BEACON_MISSED:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_BEACON_MISSED"));
            #endif
            break;
        case EV_BEACON_TRACKED:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_BEACON_TRACKED"));
            #endif
            break;
        case EV_JOINING:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_JOINING"));
            #endif
            break;
        case EV_JOINED:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_JOINED"));
            
              #ifdef MCCI_LMIC
              //This wont work with [matthijskooijman/arduino-lmic] version
              {
                u4_t netid = 0;
                devaddr_t devaddr = 0;
                u1_t nwkKey[16];
                u1_t artKey[16];
                LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
                dbgSerial.print("netid: ");
                dbgSerial.println(netid, DEC);
                dbgSerial.print("devaddr: ");
                dbgSerial.println(devaddr, HEX);
                dbgSerial.print("artKey: ");
                for (int i=0; i<sizeof(artKey); ++i) {
                  if (i != 0)
                    dbgSerial.print("-");
                  dbgSerial.print(artKey[i], HEX);
                }
                dbgSerial.println("");
                dbgSerial.print("nwkKey: ");
                for (int i=0; i<sizeof(nwkKey); ++i) {
                        if (i != 0)
                                dbgSerial.print("-");
                        dbgSerial.print(nwkKey[i], HEX);
                }
                dbgSerial.println("");
              }            
              #endif
            #endif
            
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example. (Wait what? Why? LinkCheck has no payload?? -JJ)         
            LMIC_setLinkCheckMode(0);   //uncomment 4-9 2:45 -JJ 
            LMIC_setDrTxpow(DR_SF7,14); //this does not seem to do anything at all -JJ
            break;        
        case EV_JOIN_FAILED:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_JOIN_FAILED"));
            #endif
            break;
        case EV_REJOIN_FAILED:
            //If link check enabled, we will eventually end up here. When that happens, 
            //execution essentially stops because the os_setCallback(&sendjob, do_send)
            //function is never called. UPDATE: We end up here even whenever I turn ADR
            //off, even when link check is disabled -arghhh!            
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_REJOIN_FAILED"));
            #endif

            //For right now, just call do_send again and see what happens.
            //Eventually, we may need to be a little more sophisticated
            os_setCallback (&sendjob, do_send); //<- Do this immediately
            
            break;
        case EV_TXCOMPLETE:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
              if (LMIC.txrxFlags & TXRX_ACK)
                dbgSerial.println(F("Received ack"));
            #endif

            //digitalWrite(PIN_BLUE_LED,HIGH); //Just for debugging

            //Check for a downlink
            if (LMIC.dataLen != 0) {
              #ifdef SERIAL_DEBUG
                dbgSerial.println(F("Received "));
                dbgSerial.println(LMIC.dataLen);
                dbgSerial.println(F(" bytes of payload"));
              #endif

              uint8_t dlPort = 0;
              if (LMIC.txrxFlags & TXRX_PORT){
                dlPort = LMIC.frame[LMIC.dataBeg - 1];
              }
              
              parseDownlink(dlPort, LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
            }   
            

            // Timing & Sleep Code ////////////////////////////////////////////////////

            nextAlarmSec = (nextAlarmSec + (sleepInterval % 60)) % 60;
            nextAlarmMin = (nextAlarmMin + (sleepInterval / 60)) % 60;
          
            #ifdef SERIAL_DEBUG    
              dbgSerial.print("nextAlarmSec: ");
              dbgSerial.println(nextAlarmSec);
              dbgSerial.print("nextAlarmMin: ");
              dbgSerial.println(nextAlarmMin);
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
              dbgSerial.println(F("Going to sleep now"));
              dbgSerial.flush();
            #endif 

            // This needs to be more sophisticated, just wanted to show that you 
            // can(and should) check if it's safe to go to sleep.
            while(os_queryTimeCriticalJobs(ms2osticksRound(sleepInterval * 1000))){
              dbgSerial.println(F("Waiting for time criticle job"));              
            }                
            //rtc.standbyMode(); // <-- Doesn't handle SysTick properly     
            goToSleep(); // Safe sleep function. Could probably also use ArduinoLowPower library

            adjust_millis_forward(sleepInterval * 1000);  //Fix millis() to account for time we slept - IMPORTANT! 

            #ifdef SERIAL_DEBUG
            dbgSerial.println(F("Good morning!"));
            #endif         
                      
            
            digitalWrite(LED_BUILTIN,HIGH); //Waste of power, but this is just an example 

            ////////////////////////////////////////////////////////////////////////////
            
            
            // Setup the next transmission                        
            os_setCallback (&sendjob, do_send); //<- Do this immediately  
                      
            break; 
                       
        case EV_LOST_TSYNC:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_LOST_TSYNC"));
            #endif
            break;
        case EV_RESET:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_RESET"));
            #endif
            break;
        case EV_RXCOMPLETE:
            //data received in ping slot
            //This is for Class B only 
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_RXCOMPLETE"));
            #endif
            break;
        case EV_LINK_DEAD:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_LINK_DEAD"));
            #endif
            break;
        case EV_LINK_ALIVE:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_LINK_ALIVE"));
            #endif
            break;
        
        #ifdef MCCI_LMIC
        case EV_TXSTART:
            #ifdef SERIAL_DEBUG
              dbgSerial.println(F("EV_TXSTART"));
            #endif
            break;
        #endif
        
        default:
            #ifdef SERIAL_DEBUG
              dbgSerial.print(F("Unknown event: "));
              dbgSerial.println((unsigned) ev);
            #endif
            break;
    }
}



void do_send(osjob_t* j){
  
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
      #ifdef SERIAL_DEBUG
        dbgSerial.println(F("OP_TXRXPEND, not sending"));
      #endif
    } else {
        
        // Take readings ///////////////////////////////////

        digitalWrite(PIN_EX_EN,LOW); //Turn on excitation
        
        //Get the bus voltage. The voltage returned when on battery will
        //be a little low (~0.2V) because of the Vdrop over the schottky diode.
        readings[0] = getBusVoltage();
      
        //Read AD7794 chip temperature
        readings[1] = adc.read(6);
      
        //Read 6 ADC channels
        for(int i=0; i < 6; i++){
          readings[i+2] = adc.read(i);
        }

        //Temporary scailing for testing with 2 SGs and 10 lb load cell
        readings[2] = voltsToEngUnits(readings[2], 5);
        readings[6] = voltsToMicrostrain(readings[6], 2.00);
        readings[7] = voltsToMicrostrain(readings[7], 2.00);

        digitalWrite(PIN_EX_EN,HIGH); //Turn off excitation

                
        // Send out the readings //////////////////////////        
        
        //Pack it up...
        memcpy(sendBuffer, readings, sizeof(readings)); 
        
        
        int lmicErr;
        lmicErr = LMIC_setTxData2(1, sendBuffer, sizeof(sendBuffer), 0);
        
        // We should do something useful with the returned arror code, but for 
        // the example I'll just blink out the error code returned, and try again.
        if(lmicErr) { //0 is success, anything else is a problem
          blinkError(lmicErr); 
          os_setCallback (&sendjob, do_send); //try the whole thing again
        }  

        #ifdef SERIAL_DEBUG
          dbgSerial.println(F("Packet queued"));
        #endif
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


// Parse downlink messages
// Shows 2 easy ways to interpret downlink commands from your application.
//  1> Use a "command byte". This is a simple way to have up to 255 unique
//     commands and works with other libraries that don't support ports on
//     downlinks.
//  2> Use the Port field as your command. Ports 1-223 are available for 
//     user applications and you save 1 byte of payload.
// 
// You must first setup the encode function (Device-profiles -> Codec in Chirpstack)
// for these to work.
//
void parseDownlink(uint8_t port, uint8_t * buf, uint8_t len){  
  
  // A message recieved on port 1 will be handled assuming that the first byte
  // is a "command", and the next 32 bytes are available as payload. In this 
  // example we have just on command, whitch sets the sleep/TX interval. You
  // could easily add more commands with more if() statements;
  if(port == 1){ 
    
    //Struct to represent simple message
    struct dlMessage_t{
      uint8_t   cmd;
      uint8_t   payload[32]; //Need to look into actual appropriate max payload size
    } message;
  
    memset(message.payload,0,sizeof(message.payload));  
    memcpy(&message, buf, len);
  
    //Debug info
    #ifdef SERIAL_DEBUG
      dbgSerial.print(F("Message received [ "));
      dbgSerial.print(F("port: "));
      dbgSerial.print(port);
      dbgSerial.print(F(" cmd: "));
      dbgSerial.print(message.cmd);
      dbgSerial.print(F(" Payload: "));
      for(int i=0; i<len; i++){
        dbgSerial.print(message.payload[i],HEX);
        dbgSerial.print(' ');
      }
      dbgSerial.println(" ]");
    #endif

    //Just the one command for now, but more can be added easily    
    if(message.cmd == DLCMD_SET_RATE){
      
      //We got it! 
      digitalWrite(PIN_BLUE_LED,LOW);
      delay(500);
      digitalWrite(PIN_BLUE_LED,HIGH);
      
      uint16_t arg1; 
      arg1 =  message.payload[1] | message.payload[0] << 8 ;   
      
  
      #ifdef SERIAL_DEBUG
        dbgSerial.print("arg1 = ");
        dbgSerial.println(arg1);
      #endif
         
      if((arg1 >= MIN_INTERVAL) && (arg1 < MAX_INTERVAL)){
        sleepInterval = arg1;
  
        //Just re-sync these whenever we change the interval to avoid timing issues
        nextAlarmSec = rtc.getSeconds();
        nextAlarmMin = rtc.getMinutes();
        
        #ifdef SERIAL_DEBUG
          dbgSerial.print(F("Set inteval to "));
          dbgSerial.println(arg1);
        #endif
      }    
      
    }else{
      //Catch any undefined commands
      #ifdef SERIAL_DEBUG
        dbgSerial.println(F("Unknown command"));
      #endif    
    }
  }

  // A few examples of using the port field for commands.
  // First, Port 2 will also be a command to set the sleep/TX interval
  // Send {"arg1": 60} to port 2 to change the rate to 60 seconds
  else if(port == 2){ 
    uint16_t arg1; 
    arg1 =  buf[1] | buf[0] << 8 ;

    #ifdef SERIAL_DEBUG
      dbgSerial.print("arg1 = ");
      dbgSerial.println(arg1);
    #endif
   
    //TODO: Set sleep interval here 
    if((arg1 >= MIN_INTERVAL) && (arg1 < MAX_INTERVAL)){
      sleepInterval = arg1;

      //Just re-sync these whenever we change the interval to avoid timing issues
      nextAlarmSec = rtc.getSeconds();
      nextAlarmMin = rtc.getMinutes();
      
      #ifdef SERIAL_DEBUG
        dbgSerial.print(F("Set inteval to "));
        dbgSerial.println(arg1);
      #endif
    }    
  }

  // Port 3 will be used to turn on LED for the requested interval   
  // Send {"arg1": 5000} to port 3 to light LED for 5 seconds
  else if(port == 3){ 
    uint16_t arg1; 
    arg1 =  buf[1] | buf[0] << 8 ;

    #ifdef SERIAL_DEBUG
      dbgSerial.print(F("Turning LED on for "));
      dbgSerial.print(arg1);
      dbgSerial.println(F(" mS"));
    #endif
    
    digitalWrite(PIN_BLUE_LED,LOW);
    delay(arg1);
    digitalWrite(PIN_BLUE_LED,HIGH);
    
  }

  // And Port 4 will be used to turn ADR (Adaptive data rate) on or off
  // Send {"arg1": 1} to port 4 enable ADR (1 = on, 0 = off)
  else if(port == 4){ 
    uint8_t arg1;

    arg1 = buf[0];

    if((arg1 == 0) || (arg1 == 1))
      LMIC_setAdrMode(arg1);

    #ifdef SERIAL_DEBUG
      dbgSerial.print(F("Set ADR to "));
      dbgSerial.println(arg1);
    #endif
  }
  
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

// Blink out an error code
void blinkError(int errno) {
  const uint8_t maxVal = 8; //arbitrary limit

  uint8_t i;
  for (i=0; (i < abs(errno)) || (i > maxVal) ; i++) {
    digitalWrite(PIN_BLUE_LED, LOW);
    delay(200);
    digitalWrite(PIN_BLUE_LED, HIGH);
    delay(500);    
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

//For full bridge sensor, convert to mV/V and apply linear scailing
float voltsToEngUnits(float volts,float scaleFactor){
  const float vEx = 2.5;

  float mVpV = (volts * 1000) / vEx;
  return mVpV * scaleFactor;
}

//Conversion for 1/4 bridge strain gauge
float voltsToMicrostrain(float volts,float gaugeFactor){
  //Convert readings to uE
  const float vEx = 2.5;
  const float gf = gaugeFactor;

  float Vr = volts / vEx;
  return 1E6 * ((4 * Vr) / (gf * (1 +  (2 * Vr)))); // * (1 + Rl/Rg) for lead wire resistance correction

}

//Need this to change EUI to LSB
void reverseBytes(void *buf, int size) {
    unsigned char *lo = (unsigned char*)buf;
    unsigned char *hi = (unsigned char*)buf + size - 1;
    unsigned char swap;
    while (lo < hi) {
        swap = *lo;
        *lo++ = *hi;
        *hi-- = swap;
    }
  }


//RTC Alarm Match ISR
void rtcAlarm(){
  //Do nothing for now
  
}


/***********************************************************************************
Example Javascript Decode/Encode functions for Chirpstack. This will also work for
TTN but you have to change the Decode signature slightly to Decoder(bytes, port)
  
From https://www.thethingsnetwork.org/forum/t/decode-float-sent-by-lopy-as-node/8757/2
Based on https://stackoverflow.com/a/37471538 by Ilya Bursov

************************************************************************************



//// Helper that converts incoming raw byte back to floats /////////////////////////////////////

function bytesToFloat(bytes) {
  // JavaScript bitwise operators yield a 32 bits integer, not a float.
  // Assume LSB (least significant byte first).
  var bits = bytes[3]<<24 | bytes[2]<<16 | bytes[1]<<8 | bytes[0];
  var sign = (bits>>>31 === 0) ? 1.0 : -1.0;
  var e = bits>>>23 & 0xff;
  var m = (e === 0) ? (bits & 0x7fffff)<<1 : (bits & 0x7fffff) | 0x800000;
  var f = sign * m * Math.pow(2, e - 150);
  return f;
  
  // The “bias” value, which is documented 5 to be 127 for 32 bits single-precision 
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

function Encode(port, object) {
  // Encode downlink messages sent as
  // object to an array or buffer of bytes.
  var bytes = [];
  
  // We can use Ports for different commands or functions
  if (port === 1){ //<- Port one will continue to use a "command byte"
        
    bytes[0] = object.command & 0xFF;
    
    bytes[1] = (object.arg1 >> 8) & 0xFF;
    bytes[2] = object.arg1 & 0xFF;
  }
  else if (port === 2){ //<- Port 2 will send the agument alone. The port number itself will be the command
    
    bytes[0] = (object.arg1 >> 8) & 0xFF;
    bytes[1] = object.arg1 & 0xFF;
  }
  else if (port === 3){ //<- Port 3 will turn on the blue LED for the provided number of milliseconds
    bytes[0] = (object.arg1 >> 8) & 0xFF;
    bytes[1] = object.arg1 & 0xFF;
  }
  else if (port === 4){
    bytes[0] = object.arg1;
  }
  return bytes; 
}

*****************************************************************************************/
