/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "IC_DW1000_Ranging_local.h"

#include "common.h"

void setup_tag(uint16_t short_address);
void loop_tag();

void sleepForMs(uint32_t duration);
void callbackFct();
void newRange_tag();
void setupDw1000DelayedIsr();
void dw1000RequestDelayedIsr(unsigned long delayUs);

void checkCommandMenu_tag();
