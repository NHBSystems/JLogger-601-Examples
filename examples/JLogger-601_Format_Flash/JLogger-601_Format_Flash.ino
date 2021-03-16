// Adafruit SPI Flash FatFs Format Example
// Author: Tony DiCola
//
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!  NOTE: YOU WILL ERASE ALL DATA BY RUNNING THIS SKETCH!  !!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//
// Usage:
// - Open the serial monitor at 115200 baud.  You should see a
//   prompt to confirm formatting.  If you don't see the prompt
//   close the serial monitor, press the board reset buttton,
//   wait a few seconds, then open the serial monitor again.
// - Type OK and enter to confirm the format when prompted.
// - Partitioning and formatting will take about 30-60 seconds.
//   Once formatted a message will be printed to notify you that
//   it is finished.
//
// --------------------------------------------------------------
//
// This version of the Adafruit example has been slightly modified
// for use with the JLogger-601.
//
// Changes:
// - flashTransport configured with CS pin for JLogger
// - RFM radio module CS pin set high so radio doesn't
//   interfere with SPI bus
// - This sketch erases the flash befor formatting, orginal did not
//
//  Jaimy Juliano 3-9-2021


#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

// Since SdFat doesn't fully support FAT12 such as format a new flash
// We will use Elm Cham's fatfs f_mkfs() to format
#include "ff.h"
#include "diskio.h"


Adafruit_FlashTransport_SPI flashTransport(PIN_FLASH_CS, &SPI);

Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatFileSystem fatfs;


void setup() {
  // Initialize serial port and wait for it to open before continuing.
  Serial.begin(115200);
  while (!Serial) {
    delay(100);
  }

  
  //Set RFMXX CS pin high. Still need to do this when not using radio
  pinMode(PIN_RFM_CS, OUTPUT);
  digitalWrite(PIN_RFM_CS, HIGH);
  

  
  Serial.println("Adafruit SPI Flash FatFs Format Example");

  // Initialize flash library and check its chip ID.
  if (!flash.begin()) {
    Serial.println("Error, failed to initialize flash chip!");
    while(1);
  }
  Serial.print("Flash chip JEDEC ID: 0x"); Serial.println(flash.getJEDECID(), HEX);

  // Wait for user to send OK to continue.
  Serial.setTimeout(30000);  // Increase timeout to print message less frequently.
  do {
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("This sketch will ERASE ALL DATA on the flash chip and format it with a new filesystem!");
    Serial.println("Type OK (all caps) and press enter to continue.");
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  //Wait for the magic word
  while ( !Serial.find("OK"));

  

  //Erase flash ///////////////////////////////////////////////////////////////////
  
  Serial.println("Erasing flash chip...");
  Serial.println("Let it run for ~30 seconds until the flash erase is finished.");
  Serial.println("An error or success message will be printed when complete.");

  if (!flash.eraseChip()) {
    Serial.println("Failed to erase chip!");
  }
  
  flash.waitUntilReady();
  Serial.println("Successfully erased chip!");

  Serial.println();

  

  //Format flash ////////////////////////////////////////////////////////////////////
  
  // Call fatfs begin and passed flash object to initialize file system
  Serial.println("Creating and formatting FAT filesystem...");

  // Make filesystem.
  uint8_t buf[512] = {0};          // Working buffer for f_fdisk function.    
  FRESULT r = f_mkfs("", FM_FAT | FM_SFD, 0, buf, sizeof(buf));
  if (r != FR_OK) {
    Serial.print("Error, f_mkfs failed with error code: "); Serial.println(r, DEC);
    while(1);
  }

  // sync to make sure all data is written to flash
  flash.syncBlocks();
  
  Serial.println("Formatted flash!");

  

  // Check new filesystem /////////////////////////////////////////////////////////
  
  if (!fatfs.begin(&flash)) {
    Serial.println("Error, failed to mount newly formatted filesystem!");
    while(1) delay(1);
  }

  // Done!
  Serial.println("Flash chip successfully formatted with new empty filesystem!");
}

void loop() {
  // Nothing to be done in the main loop.
}


//--------------------------------------------------------------------+
// fatfs diskio
//--------------------------------------------------------------------+
extern "C"
{

DSTATUS disk_status ( BYTE pdrv )
{
  (void) pdrv;
	return 0;
}

DSTATUS disk_initialize ( BYTE pdrv )
{
  (void) pdrv;
	return 0;
}

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
  (void) pdrv;
	return flash.readBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
  (void) pdrv;
  return flash.writeBlocks(sector, buff, count) ? RES_OK : RES_ERROR;
}

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
  (void) pdrv;

  switch ( cmd )
  {
    case CTRL_SYNC:
      flash.syncBlocks();
      return RES_OK;

    case GET_SECTOR_COUNT:
      *((DWORD*) buff) = flash.size()/512;
      return RES_OK;

    case GET_SECTOR_SIZE:
      *((WORD*) buff) = 512;
      return RES_OK;

    case GET_BLOCK_SIZE:
      *((DWORD*) buff) = 8;    // erase block size in units of sector size
      return RES_OK;

    default:
      return RES_PARERR;
  }
}

}
