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

#define EEPROM_MYCONFIG_TAG_ADDR   128

typedef struct {
  uint16_t ourAddr;
  uint16_t channel;
  uint16_t tx_power;
  
  uint16_t padding;
  uint32_t checksum;
} myConfigTag_t;

extern myConfigTag_t myConfig_tag;



bool config_store_tag();
bool config_load_tag(uint32_t defaultID);


#endif  //__CONFIG_MENU_H__
