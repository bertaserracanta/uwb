/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#ifndef __CONFIG_MENU_H__
#define __CONFIG_MENU_H__

#include <Arduino.h>
#include <EEPROM.h>
#include "crc.h"

#define EEPROM_MYCONFIG_ANC_ADDR   256

typedef struct {
  uint16_t channel;
  uint16_t tx_power;
  uint8_t outputTable;
  uint8_t weAreTag;
  uint32_t checksum;
} myConfigAnc_t;

extern myConfigAnc_t myConfig_anc;



bool config_store_anchor();
bool config_load_anchor();


#endif  //__CONFIG_MENU_H__
