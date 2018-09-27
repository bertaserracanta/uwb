/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include "config_menu_tag.h"

myConfigTag_t myConfig_tag = {
  .ourAddr =       0x1234,
  .channel =            5,
  .tx_power =         275,
  
  .padding =            0,
  .checksum =           0,
};


void config_defaults_tag(uint32_t defaultID)
{
  myConfig_tag.ourAddr =    defaultID;
  myConfig_tag.channel =            5;
  myConfig_tag.tx_power =         275;
  
  myConfig_tag.padding =            0;
  myConfig_tag.checksum =           0;
}

void config_setUnconfigurableVals_tag()
{
}

bool config_store_tag()
{
  myConfig_tag.checksum = crc_calc((uint8_t*)&myConfig_tag, sizeof(myConfig_tag)-4);
  for (unsigned long i = 0; i < sizeof(myConfig_tag); i++)
  {
    reloadWatchdog();
    noInterrupts();
    if (EEPROM.read(EEPROM_MYCONFIG_TAG_ADDR + i) != (((unsigned char *)&myConfig_tag)[i]))
      EEPROM.write(EEPROM_MYCONFIG_TAG_ADDR + i, ((unsigned char *)&myConfig_tag)[i]);
    interrupts();
  }
  return true;
}

bool config_load_tag(uint32_t defaultID)
{
  for (unsigned long i = 0; i < sizeof(myConfig_tag); i++)
  {
    reloadWatchdog();
    ((unsigned char *)&myConfig_tag)[i] = EEPROM.read(EEPROM_MYCONFIG_TAG_ADDR + i);
  }
  if ( myConfig_tag.checksum != crc_calc((uint8_t*)&myConfig_tag, sizeof(myConfig_tag)-4) )
  {
    config_defaults_tag(defaultID);
  }
  config_setUnconfigurableVals_tag();
  return true;
}
