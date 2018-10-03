/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/

#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "IC_DW1000_Ranging_local.h"
#include "mainTag.h"
#include "mainAnchor.h"


// Define your hardware setup
// 0: DW1000 USB device as Anchor (Mode toggleable via menu. See our wiki.)
// 1: DW1000 USB device forced to TagMode
// 2: DW1000 Tag
#define USED_HARDWARE 0


// On radino32 during production a serial number ist written to EEPROM at address 0x8
// This serial number will overide the SHORT_ADDRESS define below
#define SN_OFFSET 0x08


// Define the modules default short address (used for wireless addressing)
// May be changed by commandmenu on some hardware (id set by command menu has priority)
// A valid serial number in EEPROM is favored over this define
#define SHORT_ADDRESS 0x9876


// Define Antenna Delay Override for user defined calibration
//#define ANTENNA_DELAY_OVERRIDE 32850UL

uint16_t def_short_address;
uint32_t usedAntennaDelay;
void setup()
{
  //Initialize all gpios
  for (uint8_t i=0;i<BOARD_NR_GPIO_PINS;i++)
  {
    pinMode(i,INPUT);
  }
  delay(10);

  //Check if we have a valid serial number from production test
  //On L4 there is no EEPROM
  #ifdef ARDUINO_RADINO32
    uint8_t tmpdata = EEPROM.read(SN_OFFSET);
    uint8_t tmpout = 0;
    if(tmpdata == 'S')
    {
      // read short ID from EEPROM
      uint8_t i = 0;
      for(i = 0; i < 4; i++)
      {
        tmpdata = EEPROM.read(SN_OFFSET + 3 + i);
        if((tmpdata >= '0') && (tmpdata <= '9')) { tmpout = tmpdata - '0'; }
        if((tmpdata >= 'a') && (tmpdata <= 'f')) { tmpout = tmpdata - 'a' + 10; }
        if((tmpdata >= 'A') && (tmpdata <= 'F')) { tmpout = tmpdata - 'A' + 10; }
        def_short_address |= ( tmpout << (12 - (i*4)));
      }
    }
    else
    {
      def_short_address = SHORT_ADDRESS;
    }
  #endif

  #ifdef ANTENNA_DELAY_OVERRIDE
    usedAntennaDelay = ANTENNA_DELAY_OVERRIDE;
  #else
    switch (USED_HARDWARE)
    {
      default: usedAntennaDelay=0; break;
      case  0: usedAntennaDelay=32850UL; break;
    }
  #endif

  switch (USED_HARDWARE)
  {
    default: break;
    case 0: setup_anchor(def_short_address, false, usedAntennaDelay); break;
    case 1: setup_anchor(def_short_address, true, usedAntennaDelay); break;
    case 2: setup_tag(def_short_address); break;
  }
}

void loop()
{
  switch (USED_HARDWARE)
  {
    default: break;
    case 0: loop_anchor(); break;
    case 1: loop_anchor(); break;
    case 2: loop_tag(); break;
  }
}
