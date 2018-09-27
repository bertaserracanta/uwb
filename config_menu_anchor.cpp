/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include "config_menu_anchor.h"

myConfigAnc_t myConfig_anc = {
  .channel =            5,
  .tx_power =         275,
  .outputTable =        1,   //true: print device table, false: print single measurements
  .weAreTag =           0,
  .checksum =           0,
};


void config_defaults_anchor()
{
  myConfig_anc.channel =            5;
  myConfig_anc.tx_power =         275;
  myConfig_anc.outputTable =        1;   //true: print device table, false: print single measurements
  myConfig_anc.weAreTag =           0;
  myConfig_anc.checksum =           0;
}

void config_setUnconfigurableVals_anchor()
{
}

bool config_store_anchor()
{
  myConfig_anc.checksum = crc_calc((uint8_t*)&myConfig_anc, sizeof(myConfig_anc)-4);
  for (unsigned long i = 0; i < sizeof(myConfig_anc); i++)
  {
    reloadWatchdog();
    noInterrupts();
    if (EEPROM.read(EEPROM_MYCONFIG_ANC_ADDR + i) != (((unsigned char *)&myConfig_anc)[i]))
      EEPROM.write(EEPROM_MYCONFIG_ANC_ADDR + i, ((unsigned char *)&myConfig_anc)[i]);
    interrupts();
  }
  return true;
}

bool config_load_anchor()
{
  for (unsigned long i = 0; i < sizeof(myConfig_anc); i++)
  {
    reloadWatchdog();
    ((unsigned char *)&myConfig_anc)[i] = EEPROM.read(EEPROM_MYCONFIG_ANC_ADDR + i);
  }
  if ( myConfig_anc.checksum != crc_calc((uint8_t*)&myConfig_anc, sizeof(myConfig_anc)-4) )
  {
    config_defaults_anchor();
  }
  config_setUnconfigurableVals_anchor();
  return true;
}
