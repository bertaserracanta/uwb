/*
 * Copyright (c) 2015 by Thomas Trojer <thomas@trojer.net> and Leopold Sayous <leosayous@gmail.com>
 * Modified 2017 for radino32 and radinoL4 compatibility by In-Circuit GmbH
 * 
 * Decawave DW1000 library for arduino.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @file DW1000Ranging.h
 * Arduino global library (source file) working with the DW1000 library 
 * for the Decawave DW1000 UWB transceiver IC.
 */
 

#include "IC_DW1000_Ranging_local.h"
#include "ICDWR_DW1000Device_local.h"

#define MAX_DEVICES_REMOVED_PER_LOOP 2

bool weAreSleeping = true;
bool forcedSleep = false;

DW1000RangingClass DW1000Ranging;

uint8_t DW1000RangingClass::batLevel=0;
uint8_t DW1000RangingClass::lastBlinkId=0;

//other devices we are going to communicate with which are on our network:
DW1000Device DW1000RangingClass::_networkDevices[MAX_DEVICES];
byte DW1000RangingClass::_currentAddress[8];
byte DW1000RangingClass::_currentShortAddress[2];
byte DW1000RangingClass::_lastSentToShortAddress[2];
unsigned short DW1000RangingClass::_networkDevicesNumber=0;
unsigned short DW1000RangingClass::_lastDistantDevice=0;
DW1000Mac DW1000RangingClass::_globalMac;

enum ic_dw1000_deviceType DW1000RangingClass::_myDevType=IC_DW_BASIC;

//module type (anchor or tag)
int DW1000RangingClass::_type;
// message flow state
volatile byte DW1000RangingClass::_expectedMsgId;
// message sent/received state
volatile boolean DW1000RangingClass::_sentAck=false;
volatile boolean DW1000RangingClass::_receivedAck=false;
// protocol error state
boolean DW1000RangingClass::_protocolFailed=false;
// timestamps to remember
unsigned long DW1000RangingClass::timer=0;
short DW1000RangingClass::counterForBlink=0;
unsigned long DW1000RangingClass::lastForeignPacket=0;

// data buffer
byte DW1000RangingClass::data[LEN_DATA];
uint8_t DW1000RangingClass::dataToSend=LEN_DATA;
// reset line to the chip
unsigned int DW1000RangingClass::_RST;
unsigned int DW1000RangingClass::_SS;
unsigned int DW1000RangingClass::_IRQ;
unsigned int DW1000RangingClass::_WAKE;
// watchdog and reset period
unsigned long DW1000RangingClass::lastInterruptTime;
unsigned long DW1000RangingClass::_lastActivity;
unsigned long DW1000RangingClass::_resetPeriod;
// reply times (same on both sides for symm. ranging)
unsigned long DW1000RangingClass::_replyDelayTimeUS;
//timer delay
unsigned int DW1000RangingClass::_timerDelay;
// ranging counter (per second)
unsigned int DW1000RangingClass::_successRangingCount=0;
unsigned long DW1000RangingClass::_rangingCountPeriod=0;
//Here our handlers
void (*DW1000RangingClass::_handleNewRange)(void) = 0;
void (*DW1000RangingClass::_handleMeasureComplete)(void) = 0;
void (*DW1000RangingClass::_requestDelayedIsr)(unsigned long us) = 0;


/* ###########################################################################
 * #### Init and end #######################################################
 * ######################################################################### */
void DW1000RangingClass::initCommunication(unsigned int mySS, unsigned int myIRQ,unsigned int myRST,unsigned int myWAKE)
{
    // reset line to the chip
    _RST = myRST;
    _SS = mySS;
    _IRQ = myIRQ;
    _WAKE = myWAKE;
    pinMode(_WAKE, OUTPUT);
    digitalWrite(_WAKE, LOW);
    _resetPeriod = DEFAULT_RESET_PERIOD;
    // reply times (same on both sides for symm. ranging)
    _replyDelayTimeUS = DEFAULT_REPLY_DELAY_TIME;
    //we set our timer delay
    _timerDelay = DEFAULT_TIMER_DELAY;
    DW1000.begin(_SS, _IRQ, _RST);
    wakeUp(); //Wakeup in case DW1000 was put to sleep before our last reset
    DW1000.select(_SS);
}

void DW1000RangingClass::goToDeepSleep(bool preserveRegs, bool wakeToRx)
{
  wakeUp();
  if (!wakeToRx) forcedSleep=true;
  digitalWrite(_WAKE, LOW);
  DW1000.goToDeepSleep(preserveRegs, wakeToRx);
  weAreSleeping = true;
  noteActivity();
}

void DW1000RangingClass::wakeUp()
{
  forcedSleep = false;
  while(!digitalRead(_RST))
  {
    delayMicroseconds(100);
    digitalWrite(_WAKE, HIGH);
    delayMicroseconds(100);
    digitalWrite(_WAKE, LOW);
  }
  weAreSleeping = false;
  noteActivity();
}

void DW1000RangingClass::execDelayedIsr()
{
  if (forcedSleep) return;
  if (!digitalRead(_RST)) {
        digitalWrite(_WAKE, !digitalRead(_WAKE));
        (*_requestDelayedIsr)(100);
   } else {
       weAreSleeping = false;
       digitalWrite(_WAKE, LOW);
   }
}

void DW1000RangingClass::configureNetwork(unsigned int deviceAddress, unsigned int networkId, const byte mode[])
{
    // general configuration
    DW1000.idle();
    DW1000.newConfiguration();
    DW1000.setDefaults();
    DW1000.setDeviceAddress(deviceAddress);
    DW1000.setNetworkId(networkId);
    DW1000.enableMode(mode);
    DW1000.commitConfiguration();
}

void DW1000RangingClass::generalStart()
{
    // attach callback for (successfully) sent and received messages
    DW1000.attachSentHandler(handleSent);
    DW1000.attachReceivedHandler(handleReceived);
    // anchor starts in receiving mode, awaiting a ranging poll message
    
    if(DEBUG)
    {
        // DEBUG monitoring
        dw1000Serial.println("DW1000-arduino");
        // initialize the driver
        
        dw1000Serial.println("configuration..");
        // DEBUG chip info and registers pretty printed
        char msg[90];
        DW1000.getPrintableDeviceIdentifier(msg);
        dw1000Serial.print("Device ID: "); dw1000Serial.println(msg);
        DW1000.getPrintableExtendedUniqueIdentifier(msg);
        dw1000Serial.print("Unique ID: "); dw1000Serial.print(msg);
        char string[6];
        sprintf(string, "%02X:%02X", _currentShortAddress[0], _currentShortAddress[1]);
        dw1000Serial.print(" short: ");dw1000Serial.println(string);
        
        DW1000.getPrintableNetworkIdAndShortAddress(msg);
        dw1000Serial.print("Network ID & Device Address: "); dw1000Serial.println(msg);
        DW1000.getPrintableDeviceMode(msg);
        dw1000Serial.print("Device mode: "); dw1000Serial.println(msg);
    }
    
    // anchor starts in receiving mode, awaiting a ranging poll message
    receiver();
    // for first time ranging frequency computation
    _rangingCountPeriod = millis();
    
    weAreSleeping = false;
}

void DW1000RangingClass::startAsAnchor(const char address[],  const byte mode[], unsigned short myShortAddress, enum ic_dw1000_deviceType devType)
{
    _networkDevicesNumber = 0;
    //save the address
    DW1000.convertToByte(address, _currentAddress);
    //write the address on the DW1000 chip
    DW1000.setEUI(address);
    dw1000Serial.print("device address: ");
    dw1000Serial.println(address);
    _currentShortAddress[0]=((myShortAddress>>0)&0xFF);
    _currentShortAddress[1]=((myShortAddress>>8)&0xFF);
    
    //we configur the network for mac filtering
    //(device Address, network ID, frequency)
    DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
    
    //general start:
    generalStart();
    
    //defined type as anchor
    _type=ANCHOR;
    _myDevType = devType;
    
    dw1000Serial.println("### ANCHOR ###");
}

void DW1000RangingClass::startAsTag(const char address[],  const byte mode[], unsigned short myShortAddress, enum ic_dw1000_deviceType devType)
{
    _networkDevicesNumber = 0;
    //save the address
    DW1000.convertToByte(address, _currentAddress);
    //write the address on the DW1000 chip
    DW1000.setEUI(address);
    dw1000Serial.print("device address: ");
    dw1000Serial.println(address);
    _currentShortAddress[0]=((myShortAddress>>0)&0xFF);
    _currentShortAddress[1]=((myShortAddress>>8)&0xFF);
    
    //we configur the network for mac filtering
    //(device Address, network ID, frequency)
    DW1000Ranging.configureNetwork(_currentShortAddress[0]*256+_currentShortAddress[1], 0xDECA, mode);
    
    generalStart();
    //defined type as anchor
    _type=TAG;
    _myDevType = devType;
    
    dw1000Serial.println("### TAG ###");
}

bool DW1000RangingClass::addNetworkDevices(DW1000Device *device)
{
    boolean addDevice=true;
    //we test our network devices array to check we don't already have it
    for(short i=0; i<_networkDevicesNumber; i++)
    {
        if(_networkDevices[i].isShortAddressEqual(device))
        {
            //the device already exists
            return false;
        }
    }
    //todo check out of bound
    if(addDevice)
    {
        memcpy(&_networkDevices[_networkDevicesNumber], device, sizeof(DW1000Device));
        _networkDevices[_networkDevicesNumber].setIndex(_networkDevicesNumber);
        _networkDevicesNumber++;
        return true;
    }
    return false;
}

void DW1000RangingClass::removeNetworkDevices(short index){
    //if we have just 1 element
    if(_networkDevicesNumber==1){
        _networkDevicesNumber=0;
    }
    else if(index==_networkDevicesNumber-1) //if we delete the last element
    {
        _networkDevicesNumber--;
    }
    else
    {
        //we translate all the element wich are after the one we want to delete.
        for(int i=index; i<_networkDevicesNumber-1; i++)
        {
            memcpy(&_networkDevices[i], &_networkDevices[i+1], sizeof(DW1000Device));
            _networkDevices[i].setIndex(i);
        }
        _networkDevicesNumber--;
    }
}

/* ###########################################################################
 * #### Setters and Getters ##################################################
 * ######################################################################### */
DW1000Device* DW1000RangingClass::searchDistantDevice(byte shortAddress[])
{
    DW1000Device* retVal = NULL;
    //we compare the 2 bytes address with the others
    //for(int i=0; i<_networkDevicesNumber; i++)
    for(int i=_networkDevicesNumber; i>0; )
    {
        i--;
        //if(memcmp(shortAddress, _networkDevices[i].getByteShortAddress(), 2)==0)
        if (shortAddress[0]==_networkDevices[i]._shortAddress[0] && shortAddress[1]==_networkDevices[i]._shortAddress[1])
        {
          //we have found our device !
          retVal = &_networkDevices[i];
        }
    }
    return retVal;
}

DW1000Device* DW1000RangingClass::getDistantDevice()
{
    //we get the device which correspond to the message which was sent (need to be filtered by MAC address)
    return &_networkDevices[_lastDistantDevice];
}

/* ###########################################################################
 * #### Public methods #######################################################
 * ######################################################################### */

float DW1000RangingClass::getNearestRange()
{
  float curNearest = 0;
  for(int i=0; i<_networkDevicesNumber; i++)
  {
    if (_networkDevices[i].isInactive()) continue;
    if (_networkDevices[i].getRange()<=0) continue;
    if ((curNearest==0) || (_networkDevices[i].getRange()<curNearest))
      curNearest = _networkDevices[i].getRange();
  }
  return curNearest;
}

DW1000Device* DW1000RangingClass::getDeviceAtIdx(unsigned int idx)
{
  if (idx>=_networkDevicesNumber) return NULL;
  return &_networkDevices[idx];
}

void DW1000RangingClass::checkForReset()
{
    unsigned long curMillis = millis();
    if(!_sentAck && !_receivedAck)
    {
        // check if inactive
        if(curMillis - _lastActivity > _resetPeriod)
        {
            dw1000Serial.print(curMillis);
            dw1000Serial.print(" - ");
            dw1000Serial.print(_lastActivity);
            dw1000Serial.print(" = ");
            dw1000Serial.print(curMillis-_lastActivity);
            dw1000Serial.println(" -> ");
            dw1000Serial.println("RESET INACTIVE");
            resetInactive();
        }
        return;
    }
}

void DW1000RangingClass::checkForInactiveDevices()
{
    for(int i=0, j=0; j<MAX_DEVICES_REMOVED_PER_LOOP && i<_networkDevicesNumber; i++)
    {
        if(_networkDevices[i].isInactive())
        {
            dw1000Serial.print("delete inactive device: ");
            dw1000Serial.println(_networkDevices[i].getShortAddress(), HEX);
            //we need to delete the device from the array:
            removeNetworkDevices(i);
            j++;
        }
    }
}

short DW1000RangingClass::detectMessageType(byte datas[])
{
    if(datas[0]==FC_1_BLINK)
    {
        return BLINK;
    }
    else if(datas[0]==FC_1 && datas[1]==FC_2)
    {
        //we have a long MAC frame message (ranging init)
        return datas[LONG_MAC_LEN];
    }
    else if(datas[0]==FC_1 && datas[1]==FC_2_SHORT)
    {
        //we have a short mac frame message (poll, range, range report, etc..)
        return datas[SHORT_MAC_LEN];
    }
    return MSGTYPE_UNKNOWN;
}

void DW1000RangingClass::dwloop()
{
  if(_type==ANCHOR)
  {
    if (_requestDelayedIsr!=0)
    {
      if (weAreSleeping)
      {
        if ((millis()-_lastActivity)>1000)
        {
          dw1000Serial.println("Waking up.");
          wakeUp();
        }
        return;
      }
      if (_networkDevicesNumber==0 && (millis()-_lastActivity)>750)
      {
        dw1000Serial.println("No one in range. Sleep.");
        goToDeepSleep(true, true);
        return;
      }
    }
    else
    {
      checkForReset();
    }
  }
  
    if(_timerDelay==0 || (millis()-timer)>_timerDelay)
    {
      timer=millis();
      timerTick();
    }
}


/* ###########################################################################
 * #### Private methods and Handlers for transmit & Receive reply ############
 * ######################################################################### */


void DW1000RangingClass::handleSent()
{
    lastInterruptTime = millis();
    if (forcedSleep) return;
    //If there is an isr we are definitively not sleeping
    weAreSleeping = false;

    int messageType=detectMessageType(data);
    
    if(messageType!=POLL_ACK && messageType!= POLL && messageType!=RANGE && messageType!=RANGE_REPORT)
    {
        return;
    }
    
    //A msg was sent. We launch the ranging protocole when a message was sent
    if(_type==ANCHOR)
    {
        if(messageType == POLL_ACK)
        {
            DW1000Time timePollAckSent;
            DW1000.getTransmitTimestamp(timePollAckSent);
            DW1000Device *myDistantDevice=searchDistantDevice(_lastSentToShortAddress);
            if (myDistantDevice!=NULL) myDistantDevice->timePollAckSent = timePollAckSent;
        }
        if(messageType == RANGE_REPORT)
        {
          DW1000Device *myDistantDevice=searchDistantDevice(_lastSentToShortAddress);
          if (myDistantDevice!=NULL) myDistantDevice->lastAnsweredBlinkId = myDistantDevice->lastBlinkId;
          if (_requestDelayedIsr!=0)
            sleepToReceive(1000*TIMER_REC_DELAY_POLL*(myDistantDevice->knownCount-myDistantDevice->ourLastPos));
        }
    }
    else if(_type==TAG)
    {
        if(messageType == POLL)
        {
            DW1000Time timePollSent;
            DW1000.getTransmitTimestamp(timePollSent);
            //we search the device associated with the last send address
            DW1000Device *myDistantDevice=searchDistantDevice(_lastSentToShortAddress);
            //we save the value just for one device
            if (myDistantDevice!=NULL) myDistantDevice->timePollSent=timePollSent;
        }
        else if(messageType == RANGE)
        {
            DW1000Time timeRangeSent;
            DW1000.getTransmitTimestamp(timeRangeSent);
            //we search the device associated with the last send address
            DW1000Device *myDistantDevice=searchDistantDevice(_lastSentToShortAddress);
            //we save the value just for one device
            if (myDistantDevice!=NULL) myDistantDevice->timeRangeSent=timeRangeSent;
        }
    }
}

void DW1000RangingClass::handleReceived()
{
    uint32_t timeNow = millis();
    lastInterruptTime = timeNow;
    
    if (forcedSleep) return;
    //If there is an isr we are definitively not sleeping
    weAreSleeping = false;
    
    DW1000Time timeReceiveStamp;
    DW1000.getReceiveTimestamp(timeReceiveStamp);
    
    //we read the datas from the modules:
    // get message and parse
    uint16_t tmpDataLen = DW1000.getDataLength();
    DW1000.getData(data, tmpDataLen);
    
    int messageType=detectMessageType(data);
    
    //we have just received a BLINK message from tag
    if(messageType==BLINK && _type==TAG)
    {
      lastForeignPacket = timeNow;
    }
    else if(messageType==BLINK && _type==ANCHOR)
    {
        byte shortAddress[2];
        _globalMac.decodeShortBlinkFrame(data, shortAddress);
        //we crate a new device with th tag
        DW1000Device myTag(shortAddress);
        myTag.devType = (enum ic_dw1000_deviceType)data[SHORT_BLINK_LEN];
        myTag.knownCount = data[SHORT_BLINK_LEN+1];
        myTag.lastBlinkId = data[SHORT_BLINK_LEN+2];
        myTag.ourLastPos=0;
        myTag.lastBlinkTime = timeNow;
        myTag.noteActivity();
        noteActivity();
        
        if(addNetworkDevices(&myTag))
        {
            dw1000Serial.print("blink; 1 device added ! -> ");
            dw1000Serial.print(" short:");
            dw1000Serial.println(myTag.getShortAddress(), HEX);
            
            //we relpy by the transmit ranging init message
            transmitRangingInit(&myTag);
        } else {
            DW1000Device *myDistantDevice=searchDistantDevice(myTag._shortAddress);
            if (myDistantDevice!=NULL)
            {
              myDistantDevice->knownCount = data[SHORT_BLINK_LEN+1];
              myDistantDevice->lastBlinkId = data[SHORT_BLINK_LEN+2];
bool canSleep = false;
uint32_t delta = timeNow-myDistantDevice->lastBlinkTime;
if ((myDistantDevice->lastAnsweredBlinkId+1) == myDistantDevice->lastBlinkId)
{
  canSleep = true;
  for(int i=0; canSleep && i<_networkDevicesNumber; i++)
  {
    if (_networkDevices[i].isInactive()) continue;
    if ((timeNow-_networkDevices[i]._activity)<delta) continue;
    canSleep = false;
  }
}
              myDistantDevice->lastBlinkTime = timeNow;
              if (_requestDelayedIsr!=0)
              {
                if (canSleep)
                {
                  dw1000Serial.println("All up to date, sleep.");
                  sleepToReceive(500*1000);
                } else {
                  sleepToReceive(1000*(
                      (TIMER_DELAY_BLINK-(1+(millis()-timeNow)))
                    + (TIMER_REC_DELAY_POLL*( ((myDistantDevice->ourLastPos- (2*(myDistantDevice->lastBlinkId-myDistantDevice->lastAnsweredBlinkId)) )>0)?myDistantDevice->ourLastPos-(2*(myDistantDevice->lastBlinkId-myDistantDevice->lastAnsweredBlinkId)):0  ))
                  ));
                }
              }
            } else {
              //todo error handling
            }
        }
        _expectedMsgId=POLL;
    }
    else
    {
        //we have a short mac layer frame
        byte address[2];
        byte destAdress[2];
        _globalMac.decodeShortMACFrame(data, address, destAdress);
        
        //message for us?
        if (destAdress[0]!=_currentShortAddress[0] || destAdress[1]!=_currentShortAddress[1])
        {
          if (_type==TAG)
            lastForeignPacket = timeNow;
          if (messageType==POLL && _requestDelayedIsr!=0)
          {
            sleepToReceive(1000*(TIMER_REC_DELAY_POLL-1));
          }
          return;
        }
        
        //we get the device which correspond to the message which was sent (need to be filtered by MAC address)
        DW1000Device *myDistantDevice=searchDistantDevice(address);
        
        //then we proceed to range protocole
        if(_type==ANCHOR)
        {
            if(myDistantDevice==NULL)
            {
                dw1000Serial.print((address[0]>>4)&0xF,HEX);
                dw1000Serial.print((address[0]>>0)&0xF,HEX);
                dw1000Serial.print((address[1]>>4)&0xF,HEX);
                dw1000Serial.print((address[1]>>0)&0xF,HEX);
                dw1000Serial.println(" not found");
                return;
            }
            if(messageType != _expectedMsgId)
            {
                // unexpected message, start over again (except if already POLL)
                _protocolFailed = true;
            }
            if(messageType == POLL)
            {
              //we grab the replytime wich is for us
              unsigned short replyTime = 0;
              memcpy(&replyTime, data+SHORT_MAC_LEN+2, 2);
              myDistantDevice->ourLastPos = data[SHORT_MAC_LEN+4];
              myDistantDevice->knownCount = data[SHORT_MAC_LEN+5];
              
              //we configure our replyTime;
              _replyDelayTimeUS=replyTime;
              
              // on POLL we (re-)start, so no protocol failure
              _protocolFailed = false;
              
              myDistantDevice->timePollReceived = timeReceiveStamp;
              //we note activity for our device:
              myDistantDevice->noteActivity();
              //we indicate our next receive message for our ranging protocole
              _expectedMsgId = RANGE;
              transmitPollAck(myDistantDevice);
              noteActivity();
              return;
            }
            else if(messageType == RANGE)
            {
              //we grab the replytime
              myDistantDevice->timeRangeReceived = timeReceiveStamp;
              noteActivity();
              _expectedMsgId = POLL;
              
              if(!_protocolFailed)
              {
                myDistantDevice->timePollSent.setTimestamp(data+SHORT_MAC_LEN+1);
                myDistantDevice->timePollAckReceived.setTimestamp(data+SHORT_MAC_LEN+6);
                myDistantDevice->timeRangeSent.setTimestamp(data+SHORT_MAC_LEN+11);
                
                // (re-)compute range as two-way ranging is done
                DW1000Time myTOF;
                computeRangeAsymmetric(myDistantDevice, &myTOF); // CHOSEN RANGING ALGORITHM
                
                float distance=myTOF.getAsMeters();
                
                myDistantDevice->setRXPower(DW1000.getReceivePower());
                myDistantDevice->setRange(distance);
                
                myDistantDevice->setFPPower(DW1000.getFirstPathPower());
                myDistantDevice->setQuality(DW1000.getReceiveQuality());
                
                //we send the range to TAG
                transmitRangeReport(myDistantDevice);
                
                myDistantDevice->setOtherRXPower(0);
                
                //we have finished our range computation. We send the corresponding handler
                _lastDistantDevice=myDistantDevice->getIndex();
                if (distance>(0.0)&&distance<(750.0))
                  if(_handleNewRange != 0)
                    (*_handleNewRange)();
              }
              else
              {
                transmitRangeFailed(myDistantDevice);
              }
              return;
            }
        }
        else if(_type==TAG)
        {
            if(messageType==RANGING_INIT)
            {
                //we crate a new device with the anchor
                DW1000Device myAnchor(address);
                myAnchor.devType = (enum ic_dw1000_deviceType)data[SHORT_MAC_LEN+1];
                
                if(addNetworkDevices(&myAnchor))
                {
                    dw1000Serial.print(" ranging init; 1 device added ! -> ");
                    dw1000Serial.print(" short:");
                    dw1000Serial.println(myAnchor.getShortAddress(), HEX);
                }
                noteActivity();
            }
            else if(myDistantDevice==NULL)
            {
                dw1000Serial.println("Not found");
                return;
            }
            else if(messageType != _expectedMsgId)
            {
                return;
            }
            else if(messageType == POLL_ACK)
            {
                myDistantDevice->timePollAckReceived = timeReceiveStamp;
                //we note activity for our device
                myDistantDevice->noteActivity();
                myDistantDevice->batLevel = data[SHORT_MAC_LEN+1];
                timer=timeNow;
                _expectedMsgId = RANGE_REPORT;
                //and transmit the next message (range) of the ranging protocole
                transmitRange(myDistantDevice);
            }
            else if(messageType == RANGE_REPORT)
            {
                float curRange;
                memcpy(&curRange, data+1+SHORT_MAC_LEN, 4);
                float curRXPower;
                memcpy(&curRXPower, data+5+SHORT_MAC_LEN, 4);
                _lastDistantDevice=myDistantDevice->getIndex();
                
                dw1000Serial.print("Range: ");
                dw1000Serial.println(curRange);
                
                if( curRange>0.0 && curRange<750.0 )
                {
                  //we have a new range to save !
                  myDistantDevice->setRange(curRange);
                  myDistantDevice->setOtherRXPower(curRXPower);
                  myDistantDevice->setRXPower(DW1000.getReceivePower());
                  
                  //we have finished our range computation. We send the corresponding handler
                  if(_handleNewRange != 0)
                  {
                    (*_handleNewRange)();
                  }
                }
                //Ranging Cycle finished. Jump to next
                _timerDelay = 0;
            }
            else if(messageType == RANGE_FAILED)
            {
                //Ranging Cycle finished. Jump to next
                _timerDelay = 0;
                return;
            }
        }
    }
}


void DW1000RangingClass::noteActivity()
{
    // update activity timestamp, so that we do not reach "resetPeriod"
    _lastActivity = millis();
}

void DW1000RangingClass::resetInactive()
{
    //if inactive
    if(_type==ANCHOR){
        _expectedMsgId = POLL;
goToDeepSleep(true, false);
delay(25);
wakeUp();
        receiver();
    }
    noteActivity();
}

void DW1000RangingClass::timerTick()
{
  if (_type==ANCHOR)
  {
    checkForInactiveDevices();
    _timerDelay=DEFAULT_TIMER_DELAY;
    return;
  }
  static unsigned long lastMeasureStart = 0;
        if(_networkDevicesNumber>0 && counterForBlink>0 && counterForBlink<=_networkDevicesNumber)
        {
                _expectedMsgId = POLL_ACK;
                transmitPoll(&_networkDevices[counterForBlink-1]);
                counterForBlink++;
        }
        else if(counterForBlink==0)
        {
                #define MAX_CCA (30*60*1000)    //Prevent overflow
                if ((millis()-lastForeignPacket)>MAX_CCA) lastForeignPacket=millis()-MAX_CCA;
                if ( ((millis()-lastMeasureStart)<DEFAULT_TIMER_DELAY) || ((millis()-lastForeignPacket)<
                  ((((100-((millis()-lastMeasureStart)/25))>15)&&((millis()-lastMeasureStart)/25)<100)?(100-((millis()-lastMeasureStart)/25)):15)
                  ) )
                {
                  _timerDelay=TIMER_DELAY_NOWORK;
                }
                else
                {
dw1000Serial.print("Blink start: ");
dw1000Serial.print(millis()-lastMeasureStart);
dw1000Serial.print(" cca: ");
dw1000Serial.print(millis()-lastForeignPacket);
dw1000Serial.print(" chk: ");
dw1000Serial.print(100-((millis()-lastMeasureStart)/25));
dw1000Serial.println();
                  checkForInactiveDevices();
                  lastMeasureStart=millis();
                  transmitBlink();
                  counterForBlink++;
                }
        }
        else
        {
            if(_handleMeasureComplete != 0)
            {
              (*_handleMeasureComplete)();
            }
            counterForBlink=0;
            _timerDelay=DEFAULT_TIMER_DELAY;
        }
}



void DW1000RangingClass::copyShortAddress(byte address1[],byte address2[])
{
    *address1=*address2;
    *(address1+1)=*(address2+1);
}

/* ###########################################################################
 * #### Methods for ranging protocole   ######################################
 * ######################################################################### */

void DW1000RangingClass::transmitInit()
{
    DW1000.newTransmit();
    DW1000.setDefaults();
}


void DW1000RangingClass::transmit(byte datas[])
{
    DW1000.setData(datas, dataToSend);
    dataToSend = LEN_DATA;
    DW1000.startTransmit();
}


void DW1000RangingClass::transmit(byte datas[], DW1000Time time)
{
    DW1000.setDelay(time);
    DW1000.setData(datas, dataToSend);
    dataToSend = LEN_DATA;
    DW1000.startTransmit();
}

void DW1000RangingClass::transmitBlink()
{
    _timerDelay=TIMER_DELAY_BLINK;
    transmitInit();
    _globalMac.generateShortBlinkFrame(data, _currentShortAddress);
    data[SHORT_BLINK_LEN] = _myDevType;
    data[SHORT_BLINK_LEN+1] = _networkDevicesNumber;
    data[SHORT_BLINK_LEN+2] = ++lastBlinkId;
    dataToSend = SHORT_BLINK_LEN+3;
    transmit(data);
}

void DW1000RangingClass::transmitRangingInit(DW1000Device *myDistantDevice)
{
    transmitInit();
    //we generate the mac frame for a ranging init message
    _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
    //we define the function code
    data[SHORT_MAC_LEN]=RANGING_INIT;
    data[SHORT_MAC_LEN+1] = _myDevType;
    dataToSend = SHORT_MAC_LEN+2;
    copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
    uint16_t myDelay;
    myDelay = _currentShortAddress[1]&0x07;
    myDelay <<= 8;
    myDelay += _currentShortAddress[0];
    myDelay *= 4;
myDelay += 1500;  //communication to dw1000 is too slow.
    DW1000Time deltaTime = DW1000Time(myDelay, DW_MICROSECONDS);
    transmit(data, deltaTime);
}

void DW1000RangingClass::transmitPoll(DW1000Device *myDistantDevice)
{
    transmitInit();
    
    if(myDistantDevice==NULL)
    {
        //we need to set our timerDelay:
        _timerDelay=DEFAULT_TIMER_DELAY+(int)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
        
        byte shortBroadcast[2]={0xFF, 0xFF};
        _globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
        data[SHORT_MAC_LEN] = POLL;
        //we enter the number of devices
        data[SHORT_MAC_LEN+1]=_networkDevicesNumber;
        
        for(short i=0; i<_networkDevicesNumber; i++)
        {
            //each devices have a different reply delay time.
            _networkDevices[i].setReplyTime((2*i+1)*DEFAULT_REPLY_DELAY_TIME);
            //we write the short address of our device:
            memcpy(data+SHORT_MAC_LEN+2+4*i, _networkDevices[i].getByteShortAddress(), 2);
            
            //we add the replyTime
            unsigned short replyTime=_networkDevices[i].getReplyTime();
            memcpy(data+SHORT_MAC_LEN+2+2+4*i, &replyTime, 2);
            
        }
        
        copyShortAddress(_lastSentToShortAddress,shortBroadcast);
        
    }
    else
    {
        //we redefine our default_timer_delay for just 1 device;
        _timerDelay=TIMER_DELAY_POLL;
        myDistantDevice->setReplyTime(DEFAULT_REPLY_DELAY_TIME);
        
        _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
        
        data[SHORT_MAC_LEN] = POLL;
        data[SHORT_MAC_LEN+1]=1;
        unsigned short replyTime=myDistantDevice->getReplyTime();
        memcpy(data+SHORT_MAC_LEN+2, &replyTime, 2);
        data[SHORT_MAC_LEN+4] = myDistantDevice->getIndex();
        data[SHORT_MAC_LEN+5] = _networkDevicesNumber;
        
        dataToSend = SHORT_MAC_LEN+6;
        
        copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
    }
    transmit(data);
}


void DW1000RangingClass::transmitPollAck(DW1000Device *myDistantDevice) {
    transmitInit();
    _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
    data[SHORT_MAC_LEN] = POLL_ACK;
    data[SHORT_MAC_LEN+1] = batLevel;
    dataToSend = SHORT_MAC_LEN+2;
    // delay the same amount as ranging tag
    DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW_MICROSECONDS);
    copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
    transmit(data, deltaTime);
}

void DW1000RangingClass::transmitRange(DW1000Device *myDistantDevice) {
    //transmit range need to accept broadcast for multiple anchor
    transmitInit();
    
    if(myDistantDevice==NULL)
    {
        //we need to set our timerDelay:
        _timerDelay=DEFAULT_TIMER_DELAY+(int)(_networkDevicesNumber*3*DEFAULT_REPLY_DELAY_TIME/1000);
        
        byte shortBroadcast[2]={0xFF, 0xFF};
        _globalMac.generateShortMACFrame(data, _currentShortAddress, shortBroadcast);
        data[SHORT_MAC_LEN] = RANGE;
        //we enter the number of devices
        data[SHORT_MAC_LEN+1]=_networkDevicesNumber;
        
        // delay sending the message and remember expected future sent timestamp
        DW1000Time deltaTime = DW1000Time(DEFAULT_REPLY_DELAY_TIME, DW_MICROSECONDS);
        DW1000Time timeRangeSent = DW1000.setDelay(deltaTime);
        
        for(short i=0; i<_networkDevicesNumber; i++)
        {
            //we write the short address of our device:
            memcpy(data+SHORT_MAC_LEN+2+17*i, _networkDevices[i].getByteShortAddress(), 2);
            
            
            //we get the device which correspond to the message which was sent (need to be filtered by MAC address)
            _networkDevices[i].timeRangeSent = timeRangeSent;
            _networkDevices[i].timePollSent.getTimestamp(data+SHORT_MAC_LEN+4+17*i);
            _networkDevices[i].timePollAckReceived.getTimestamp(data+SHORT_MAC_LEN+9+17*i);
            _networkDevices[i].timeRangeSent.getTimestamp(data+SHORT_MAC_LEN+14+17*i);
        }
        
        copyShortAddress(_lastSentToShortAddress, shortBroadcast);
        
    }
    else
    {
        _timerDelay=TIMER_DELAY_RANGE;
        
        _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
        data[SHORT_MAC_LEN] = RANGE;
        // delay sending the message and remember expected future sent timestamp
        DW1000Time deltaTime = DW1000Time(_replyDelayTimeUS, DW_MICROSECONDS);
        //we get the device which correspond to the message which was sent (need to be filtered by MAC address)
        myDistantDevice->timeRangeSent = DW1000.setDelay(deltaTime);
        myDistantDevice->timePollSent.getTimestamp(data+1+SHORT_MAC_LEN);
        myDistantDevice->timePollAckReceived.getTimestamp(data+6+SHORT_MAC_LEN);
        myDistantDevice->timeRangeSent.getTimestamp(data+11+SHORT_MAC_LEN);
        dataToSend = SHORT_MAC_LEN+16;
        copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
    }
    transmit(data);
}


void DW1000RangingClass::transmitRangeReport(DW1000Device *myDistantDevice) {
    transmitInit();
    _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
    data[SHORT_MAC_LEN] = RANGE_REPORT;
    // write final ranging result
    float curRange=myDistantDevice->getRange();
    float curRXPower=myDistantDevice->getRXPower();
    //We add the Range and then the RXPower
    memcpy(data+1+SHORT_MAC_LEN, &curRange, 4);
    memcpy(data+5+SHORT_MAC_LEN, &curRXPower, 4);
    dataToSend = SHORT_MAC_LEN+9;
    copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
//prevents freezes
delayMicroseconds(250);
    transmit(data);
}

void DW1000RangingClass::transmitRangeFailed(DW1000Device *myDistantDevice) {
    transmitInit();
    _globalMac.generateShortMACFrame(data, _currentShortAddress, myDistantDevice->getByteShortAddress());
    data[SHORT_MAC_LEN] = RANGE_FAILED;
    dataToSend = SHORT_MAC_LEN+1;
    copyShortAddress(_lastSentToShortAddress,myDistantDevice->getByteShortAddress());
    transmit(data);
}

void DW1000RangingClass::receiver() {
    DW1000.newReceive();
    DW1000.setDefaults();
    // so we don't need to restart the receiver manually
    DW1000.receivePermanently(true);
    DW1000.startReceive();
}

void DW1000RangingClass::delayedReceive(long myDelay)
{
    DW1000Time deltaTime = DW1000Time(myDelay, DW_MICROSECONDS);
    DW1000.newReceive();
    if (myDelay>1500)
      DW1000.setDelay(deltaTime);
    DW1000.startReceive();
    noteActivity();
}

void DW1000RangingClass::sleepToReceive(unsigned long myDelay)
{
    //make sure wakeup is low
    digitalWrite(_WAKE, LOW);
    if (myDelay<5500 || _requestDelayedIsr==0)
      delayedReceive(myDelay);
    else
    {
      //DW1000 needs up to 4.0 ms for wakeup procedure
      myDelay-=4500;
      weAreSleeping = true;
      (*_requestDelayedIsr)(myDelay);
      DW1000.goToDeepSleep(true, true);
    }
    noteActivity();
}





/* ###########################################################################
 * #### Methods for range computation and corrections  #######################
 * ######################################################################### */


void DW1000RangingClass::computeRangeAsymmetric(DW1000Device *myDistantDevice, DW1000Time *myTOF)
{
    // asymmetric two-way ranging (more computation intense, less error prone)
    DW1000Time round1 = (myDistantDevice->timePollAckReceived-myDistantDevice->timePollSent).wrap();
    DW1000Time reply1 = (myDistantDevice->timePollAckSent-myDistantDevice->timePollReceived).wrap();
    DW1000Time round2 = (myDistantDevice->timeRangeReceived-myDistantDevice->timePollAckSent).wrap();
    DW1000Time reply2 = (myDistantDevice->timeRangeSent-myDistantDevice->timePollAckReceived).wrap();
    
    myTOF->setTimestamp((round1 * round2 - reply1 * reply2) / (round1 + round2 + reply1 + reply2));
    //myTOF->setTimestamp(((round1-reply1)+(round2-reply2))*0.25);
    
    /* Do not use in isr!!
    dw1000Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
    dw1000Serial.print("timePollSent ");myDistantDevice->timePollSent.print();
    dw1000Serial.print("round1 "); dw1000Serial.println((long)round1.getTimestamp());
    
    dw1000Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
    dw1000Serial.print("timePollReceived ");myDistantDevice->timePollReceived.print();
    dw1000Serial.print("reply1 "); dw1000Serial.println((long)reply1.getTimestamp());
    
    dw1000Serial.print("timeRangeReceived ");myDistantDevice->timeRangeReceived.print();
    dw1000Serial.print("timePollAckSent ");myDistantDevice->timePollAckSent.print();
    dw1000Serial.print("round2 "); dw1000Serial.println((long)round2.getTimestamp());
    
    dw1000Serial.print("timeRangeSent ");myDistantDevice->timeRangeSent.print();
    dw1000Serial.print("timePollAckReceived ");myDistantDevice->timePollAckReceived.print();
    dw1000Serial.print("reply2 "); dw1000Serial.println((long)reply2.getTimestamp());
    */
}


/* FOR DEBUGGING*/
void DW1000RangingClass::visualizeDatas(byte datas[]){
    char string[60];
    sprintf(string, "%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X",
            datas[0], datas[1], datas[2], datas[3], datas[4], datas[5], datas[6], datas[7],datas[8],datas[9],datas[10],datas[11],datas[12],datas[13],datas[14],datas[15]);
    dw1000Serial.println(string);
}
