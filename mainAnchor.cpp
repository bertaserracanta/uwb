/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include "mainAnchor.h"

#include <EEPROM.h>
#include <SPI.h>
#include "IC_DW1000_Ranging_local.h"
#include "ICDWR_DW1000Device_local.h"
#include "config_menu_anchor.h"

/* Kalibrierung:
  Anker bespielen mit "#define DO_ANTENNA_CALIB 1"
  Tag in exakt DO_ANTENNA_CALIB_VALUE Meter Abstand aufstellen
  Oben rechts neben der auf Serial ausgegebenen Tabelle wird ermitteltes AntennenDelay angezeigt
  Das Antennendelay wenn es dann stabil ist, unten eintragen
    - #define ANTENNA_DELAY 32850UL
  Anker mit #define DO_ANTENNA_CALIB 0 endgueltig programmieren
*/
#define DO_ANTENNA_CALIB 0
#define DO_ANTENNA_CALIB_VALUE 3.0
//#define ANTENNA_DELAY 32850UL //DW1000 USB Stick

#ifndef ANTENNA_DELAY
  #define ANTENNA_DELAY 32850UL //DW1000 USB Stick
#endif

#define MY_DEVICE_TYPE_A     IC_DW_USB_STICK_ANC
#define MY_DEVICE_TYPE_T     IC_DW_USB_STICK_TAG

//set tx power in dBm, 0 to 33.5, step 0.5
//overides configured value (myConfig_anc.tx_power)
//Respect you local wireless regulations!
//Influences distance mesurements
//Recalibrate antenna delay after change
//#define DW_TX_POWER      33.5
//#define DW_TX_POWER      27.5
//#define DW_TX_POWER      22.5
//#define DW_TX_POWER      12.5
//#define DW_TX_POWER      0.0

//LED toggles on successful measurement. Set to 0xFF to deactivate functionality.
#define PIN_LED 13

//Where to printout information
#define SerialOut Serial
#define PIN_TXEN 0xFF //Set to TXEN for RS485 connection
#define SerialOutSpeed 57600
#define SerialOutBegin() do{SerialOut.begin(SerialOutSpeed);}while(0)

uint32_t my_antenna_delay = ANTENNA_DELAY;
uint16_t my_short_address = 0x0000;
bool weAreAnchor = true;
bool dw1000RestartRequested = false;
void setup_anchor(uint16_t userShortAddress, bool startAsTag, uint32_t userAntennaDelay)
{
  my_short_address = userShortAddress;
  if (PIN_LED != 0xFF) pinLow(PIN_LED, OUTPUT);

  SerialOutBegin();
  #if PIN_TXEN!=0xFF
    pinLow(PIN_TXEN, OUTPUT);
  #endif

  startWatchdog();

  config_load_anchor();

  if (startAsTag) myConfig_anc.weAreTag=true;
  if (myConfig_anc.weAreTag) weAreAnchor=false;
  if (userAntennaDelay!=0xFFFFFFFF) my_antenna_delay = userAntennaDelay;

  //init dw1000 configuration
  DW1000Ranging.initCommunication();
  DW1000Ranging.attachNewRange(newRange_anchor);
  DW1000Ranging.attachMeasureComplete(measureComplete);

  if (weAreAnchor) {
    switch(myConfig_anc.channel)
    {
      default:
      case 1: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_A); break;
      case 2: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_A); break;
      case 3: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_A); break;
      case 4: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_A); break;
      case 5: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_A); break;
      case 7: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_A); break;
    }
  } else {
    switch(myConfig_anc.channel)
    {
      default:
      case 1: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_T); break;
      case 2: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_T); break;
      case 3: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_T); break;
      case 4: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_T); break;
      case 5: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_T); break;
      case 7: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_T); break;
    }
  }

#ifdef DW_TX_POWER
  myConfig_anc.tx_power = (DW_TX_POWER*10);
#endif
  DW1000.setManualTxPower(myConfig_anc.tx_power);
  if (weAreAnchor)
    DW1000.setAntennaDelay(my_antenna_delay);
  else
    DW1000.setAntennaDelay(0);

  DW1000Ranging.lastInterruptTime = millis();

  //SerialOut.println("init done");
}

volatile uint32_t led_flashStart = 0;
bool cmdMode = false;  //is command mode active?
void loop_anchor()
{
  reloadWatchdog();

  checkCommandMenu_anchor();

  DW1000Ranging.dwloop();

  //At least the blink isr should happen. If not maybe DW1000 froze.
  if ( dw1000RestartRequested || (millis()-DW1000Ranging.lastInterruptTime)>(weAreAnchor?1000:5000) )
  {
    dw1000RestartRequested = false;
    DW1000Ranging.lastInterruptTime = millis();
    //reinit the configuration
    DW1000Ranging.initCommunication();
    DW1000Ranging.attachNewRange(newRange_anchor);
    DW1000Ranging.attachMeasureComplete(measureComplete);
    if (weAreAnchor) {
      switch(myConfig_anc.channel)
      {
        default:
        case 1: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_A); break;
        case 2: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_A); break;
        case 3: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_A); break;
        case 4: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_A); break;
        case 5: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_A); break;
        case 7: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_A); break;
      }
    } else {
      switch(myConfig_anc.channel)
      {
        default:
        case 1: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_T); break;
        case 2: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_T); break;
        case 3: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_T); break;
        case 4: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_T); break;
        case 5: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_T); break;
        case 7: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_T); break;
      }
    }
    DW1000.setManualTxPower(myConfig_anc.tx_power);
    if (weAreAnchor)
      DW1000.setAntennaDelay(my_antenna_delay);
    else
      DW1000.setAntennaDelay(0);
  }

  //release RS485 TXEN if buffer empty
  #if PIN_TXEN != 0xFF
    if ( !rb_full_count(SerialOut.get_tx_rb()) )
    {
      SerialOut.flush();
      digitalWrite(PIN_TXEN, LOW);
    }
  #endif

  if (PIN_LED != 0xFF && (millis() - led_flashStart) > 25 )
  {
    digitalWrite(PIN_LED, LOW);
  }
}

#if DO_ANTENNA_CALIB
unsigned short curDelay = ANTENNA_DELAY;
float delayAverage = ANTENNA_DELAY;
#endif
void printDeviceTable()
{
  #if PIN_TXEN!=0xFF
    digitalWrite(PIN_TXEN, HIGH);
  #endif
  SerialOut.print("\x1B[H");  //cursor top left
  SerialOut.print("\x1B[2J"); //clear screen
//  SerialOut.print("Active");
//  SerialOut.print("\t");
  SerialOut.print("Type");
  SerialOut.print("\t");
  SerialOut.print("Address");
  SerialOut.print("\t");
  SerialOut.print("Range");
  SerialOut.print("\t");
  SerialOut.print("Battery");
  SerialOut.print("\t");
  SerialOut.print("OthRxP");
  SerialOut.print("\t");
  SerialOut.print("OurRxP");
  SerialOut.print("  (ADDR=");
  SerialOut.print(my_short_address, HEX);
  SerialOut.print(") ");
  SerialOut.print(millis() & 0xF, HEX);
#if DO_ANTENNA_CALIB
  if (weAreAnchor)
  {
    SerialOut.print(" - ");
    SerialOut.print(delayAverage);
  }
#endif
  SerialOut.print("\r\n");

  unsigned int idx = 0;
  for ( DW1000Device* devicePtr = DW1000Ranging.getDeviceAtIdx(idx++) ; devicePtr != NULL ; devicePtr = DW1000Ranging.getDeviceAtIdx(idx++) )
  {
//    SerialOut.print(!devicePtr->isInactive());
//    SerialOut.print("\t");
    SerialOut.print(devicePtr->devType);
    SerialOut.print("\t");
    SerialOut.print(devicePtr->getShortAddress(), HEX);
    SerialOut.print("\t");
    SerialOut.print(devicePtr->getRange()); SerialOut.print(" m");
    SerialOut.print("\t");
    SerialOut.print(2+(((float)devicePtr->batLevel)/100)); SerialOut.print(" V");
    SerialOut.print("\t");
    SerialOut.print(devicePtr->getOtherRXPower()); SerialOut.print(" dBm");
    SerialOut.print("\t");
    SerialOut.print(devicePtr->getRXPower()); SerialOut.print(" dBm");
//    SerialOut.print("\t");
//    SerialOut.print(millis()-devicePtr->_activity);
    SerialOut.print("\r\n");
  }
}

void measureComplete()
{
  if ( !cmdMode && myConfig_anc.outputTable )
  {
    if (weAreAnchor)
    {
      printDeviceTable();
    }
  }
}

// Called from ISR -> Keep short!!
void newRange_anchor()
{
#if DO_ANTENNA_CALIB
  if (weAreAnchor)
  {
    if (DW1000Ranging.getDistantDevice()->getRange() > DO_ANTENNA_CALIB_VALUE)
    {
      DW1000.setAntennaDelay(++curDelay);
    } else {
      DW1000.setAntennaDelay(--curDelay);
    }
    delayAverage *= 19;
    delayAverage += curDelay;
    delayAverage /= 20;
  }
#endif

  if (PIN_LED != 0xFF) {
    led_flashStart = millis();
    digitalWrite(PIN_LED, HIGH);
  }
  if ( !cmdMode && (!weAreAnchor || !myConfig_anc.outputTable) )
  {
    #if PIN_TXEN!=0xFF
      digitalWrite(PIN_TXEN, HIGH);
    #endif
    SerialOut.print("$RAN,");
    SerialOut.print(millis()); SerialOut.print(",");
    SerialOut.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX); SerialOut.print(",");
    SerialOut.print(DW1000Ranging.getDistantDevice()->getRange()); SerialOut.print(",");
    SerialOut.print(2+(((float)DW1000Ranging.getDistantDevice()->batLevel)/100)); SerialOut.print(",");
    SerialOut.print(DW1000Ranging.getDistantDevice()->getOtherRXPower()); SerialOut.print(",");
    SerialOut.print(DW1000Ranging.getDistantDevice()->getRXPower());
    SerialOut.println();
  }
}

#define cmdSerial SerialOut
#define cmdSerialSpeed SerialOutSpeed
void checkCommandMenu_anchor()
{
  static unsigned long cmdMenuOutput = 0;  //was the information line printed
  static unsigned char cmdBuf[4];  //buffer for incoming chars
  static unsigned char cmdBufPos = 0;  //where in the buffer to store next character
  unsigned char curChar = '\0';
  unsigned short * valToModify = NULL;
  bool cmdOk = false;

  while (cmdSerial.available())
  {
    curChar = cmdSerial.read();
    if (curChar != '\r' && curChar != '\n')
    {
      //Shift char into buf
      if (cmdBufPos >= 4)
      {
        cmdBuf[0] = cmdBuf[1];
        cmdBuf[1] = cmdBuf[2];
        cmdBuf[2] = cmdBuf[3];
        cmdBuf[3] = curChar;
      }
      else
      {
        cmdBuf[cmdBufPos++] = curChar;
      }
    }
    else
    {
      //We got a line ending -> react
      #if PIN_TXEN!=0xFF
        digitalWrite(PIN_TXEN, HIGH);
      #endif
      //Trash remaining chars (e.g. Windows \r\n)
      delay(((cmdSerialSpeed / 8) / 1000) + 1);
      while (cmdSerial.available()) cmdSerial.read();

      //print out current buf content
      cmdSerial.write(cmdBuf[0]);
      cmdSerial.write(cmdBuf[1]);
      cmdSerial.write(cmdBuf[2]);
      cmdSerial.write(cmdBuf[3]);
      cmdSerial.println("   ");
      cmdSerial.write(cmdBuf[0]);
      cmdSerial.print(": ");
      valToModify = NULL;
      cmdOk = false;

      //Start command handling
      switch (cmdBuf[0])
      {
        case'f': case'F': cmdSerial.print("Forcing tag mode"); if (!cmdMode) break; cmdOk = true; dw1000RestartRequested=true; myConfig_anc.weAreTag = myConfig_anc.weAreTag?0:1; weAreAnchor=!myConfig_anc.weAreTag; break;
        case'r': case'R': cmdSerial.print("Reload config from EEPROM"); if (!cmdMode) break; cmdOk = true; config_load_anchor(); break;
        case'c': case'C': cmdSerial.print("Set Channel"); if (!cmdMode) break; valToModify = &myConfig_anc.channel; break;
        case'x': case'X': cmdSerial.print("Set TX strength"); if (!cmdMode) break; valToModify = &myConfig_anc.tx_power; break;
        case't': case'T': cmdSerial.print("Toggle output format"); cmdOk = true; myConfig_anc.outputTable = myConfig_anc.outputTable?0:1; break;
        case'q': case'Q': cmdSerial.print("Quitting command mode"); if (!cmdMode) break; cmdOk = true; cmdMode = false; break;
        case'm': case'M': cmdSerial.print("Entering command mode"); if (cmdMode) break; cmdOk = true; cmdMode = true; break;
        case's': case'S': cmdSerial.print("Storing config to EEPROM"); if (!cmdMode) break; cmdOk = true; config_store_anchor(); break;
        default: cmdSerial.print("Unknown command"); break;
      }
      cmdSerial.println("   ");
      if (valToModify != NULL)
      {
        if (cmdBufPos < 2)  //got only single char command
        {
          cmdSerial.print("No value given");
        }
        else
        {
          myConfigAnc_t confBack = myConfig_anc;
          cmdSerial.write(cmdBuf[1]);
          if (cmdBufPos > 2) cmdSerial.write(cmdBuf[2]);
          if (cmdBufPos > 3) cmdSerial.write(cmdBuf[3]);
          cmdSerial.print(": ");
          if ( !isdigit(cmdBuf[1]) || (cmdBufPos > 2 && !isdigit(cmdBuf[2])) || (cmdBufPos > 3 && !isdigit(cmdBuf[3])) )
          {
            cmdSerial.print("Invalid value");  //got non hex param
          } else {
            *valToModify = fromdigitch(cmdBuf[1]);
            if (cmdBufPos > 2)
            {
              *valToModify *= 10;
              *valToModify += fromdigitch(cmdBuf[2]);
            }
            if (cmdBufPos > 3)
            {
              *valToModify *= 10;
              *valToModify += fromdigitch(cmdBuf[3]);
            }
            
            if (myConfig_anc.channel<1) myConfig_anc.channel=5;
            if (myConfig_anc.channel>7) myConfig_anc.channel=5;
            if (myConfig_anc.channel==6) myConfig_anc.channel=5;
            if (myConfig_anc.tx_power>335) myConfig_anc.tx_power=335;
            myConfig_anc.tx_power /= 5;
            myConfig_anc.tx_power *= 5;
            cmdSerial.print("New value: ");
            cmdSerial.print(*valToModify);
            cmdOk = true;
            
            if ( myConfig_anc.channel != confBack.channel )
            {
              if (weAreAnchor) {
                switch(myConfig_anc.channel)
                {
                  default:
                  case 1: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_A); break;
                  case 2: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_A); break;
                  case 3: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_A); break;
                  case 4: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_A); break;
                  case 5: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_A); break;
                  case 7: DW1000Ranging.startAsTag("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_A); break;
                }
              } else {
                switch(myConfig_anc.channel)
                {
                  default:
                  case 1: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, my_short_address, MY_DEVICE_TYPE_T); break;
                  case 2: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, my_short_address, MY_DEVICE_TYPE_T); break;
                  case 3: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, my_short_address, MY_DEVICE_TYPE_T); break;
                  case 4: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, my_short_address, MY_DEVICE_TYPE_T); break;
                  case 5: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, my_short_address, MY_DEVICE_TYPE_T); break;
                  case 7: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, my_short_address, MY_DEVICE_TYPE_T); break;
                }
              }
              DW1000.setManualTxPower(myConfig_anc.tx_power);
              if (weAreAnchor)
                DW1000.setAntennaDelay(my_antenna_delay);
              else
                DW1000.setAntennaDelay(0);
            }
            else if (myConfig_anc.tx_power != confBack.tx_power)
            {
              DW1000.setManualTxPower(myConfig_anc.tx_power);
            }
          }
        }
        cmdSerial.println("   ");
      }
      if (cmdOk)  //command handled successfully
        cmdSerial.println("OK");
      else  //error occured during command handling
        cmdSerial.println("ERR");
      cmdMenuOutput = cmdBuf[0] = cmdBuf[1] = cmdBuf[2] = cmdBuf[3] = cmdBufPos = 0;
    }
  }
  if (cmdMode && cmdMenuOutput == 0) //a printout of the info line was scheduled
  {
    #if PIN_TXEN!=0xFF
      digitalWrite(PIN_TXEN, HIGH);
    #endif
    cmdMenuOutput = 1;
    cmdSerial.print("F)orceTag:");
    cmdSerial.print(myConfig_anc.weAreTag);
    cmdSerial.print("  C)han:");
    cmdSerial.print(myConfig_anc.channel);
    cmdSerial.print("  X)Str:");
    cmdSerial.print(myConfig_anc.tx_power / 10); cmdSerial.print("."); cmdSerial.print(myConfig_anc.tx_power % 10);
    cmdSerial.print("  T)ableOut:");
    cmdSerial.print(myConfig_anc.outputTable ? "Y" : "N");
    cmdSerial.print("  R)eload");
    cmdSerial.print("  S)tore");
    cmdSerial.print("  Q)uit");
    cmdSerial.print("\r\n> ");
  }
}
