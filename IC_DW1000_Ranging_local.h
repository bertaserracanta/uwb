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
 * Arduino global library (header file) working with the DW1000 library
 * for the Decawave DW1000 UWB transceiver IC.
 */

#ifndef __IC_DW1000_Ranging_h__
#define __IC_DW1000_Ranging_h__

#include "ICDWR_DW1000_local.h"
#include "ICDWR_DW1000Time_local.h"
#include "ICDWR_DW1000Device_local.h" 
#include "ICDWR_DW1000Mac_local.h"

// messages used in the ranging protocol
#define POLL 0
#define POLL_ACK 1
#define RANGE 2
#define RANGE_REPORT 3
#define RANGE_FAILED 255
#define BLINK 4
#define RANGING_INIT 5
#define MSGTYPE_UNKNOWN 254

#define LEN_DATA 116

//Max devices we put in the networkDevices array!
//Keep under u8 capacity
//#define MAX_DEVICES 6
#define MAX_DEVICES 64

//Default Pin for module:
#define DEFAULT_RST_PIN 28
#define DEFAULT_SPI_SS_PIN 29
#define DEFAULT_SPI_IRQ_PIN 33
#define DEFAULT_WAKE_PIN 32

#define TIME_CONSTANT 1

//Default value
//in ms
#define DEFAULT_RESET_PERIOD 750
//in us
//#define DEFAULT_REPLY_DELAY_TIME 15000UL
//#define DEFAULT_REPLY_DELAY_TIME 12500UL
//#define DEFAULT_REPLY_DELAY_TIME 10000UL
//#define DEFAULT_REPLY_DELAY_TIME 7500UL
//#define DEFAULT_REPLY_DELAY_TIME 7000UL
//#define DEFAULT_REPLY_DELAY_TIME 5000UL
//#define DEFAULT_REPLY_DELAY_TIME 2500UL
#define DEFAULT_REPLY_DELAY_TIME TIME_CONSTANT*1500UL		//any shorter leads to calculation errors
//#define DEFAULT_REPLY_DELAY_TIME 500UL

//sketch type (anchor or tag)
#define TAG 0
#define ANCHOR 1

//default timer delay
#define TIMER_DELAY_NOWORK  0    //while waiting for channel free
#define TIMER_DELAY_BLINK  TIME_CONSTANT*15    //wait this long for range init
#define TIMER_DELAY_POLL   TIME_CONSTANT*10    //wait this long for poll ack
#define TIMER_DELAY_RANGE  TIME_CONSTANT*10    //wait this long for range report
#define TIMER_REC_DELAY_POLL TIME_CONSTANT*9
#define DEFAULT_TIMER_DELAY 0  //on Anchor: time between device table cleanup, on tag: pause after cycle complete
 

//debug mode
#ifndef DEBUG
#define DEBUG true //false
#endif


class DW1000RangingClass {
  public:
    static uint8_t batLevel;
    static uint8_t lastBlinkId;
    static unsigned long lastInterruptTime;
    
    //variables 
    // data buffer
    static byte data[LEN_DATA];
    static uint8_t dataToSend;
    static enum ic_dw1000_deviceType _myDevType;
    static void delayedReceive(long myDelay);
    static void sleepToReceive(unsigned long myDelay);
    static void goToDeepSleep(bool preserveRegs, bool wakeToRx);
    static void wakeUp();
    
    static void (*_requestDelayedIsr)(unsigned long us);
    static void attachRequestDelayedIsr(void (*requestDelayedIsr)(unsigned long)) { _requestDelayedIsr = requestDelayedIsr; }; 
    static void execDelayedIsr();
    
    
    //initialisation
    static void initCommunication(unsigned int mySS=DEFAULT_SPI_SS_PIN, unsigned int myIRQ=DEFAULT_SPI_IRQ_PIN, unsigned int myRST=DEFAULT_RST_PIN, unsigned int myWAKE=DEFAULT_WAKE_PIN );
    static void configureNetwork(unsigned int deviceAddress, unsigned int networkId, const byte mode[]);
    static void generalStart();
    static void startAsAnchor(const char address[], const byte mode[], unsigned short myShortAddress, enum ic_dw1000_deviceType devType);
    static void startAsTag(const char address[], const byte mode[], unsigned short myShortAddress, enum ic_dw1000_deviceType devType);
    static bool addNetworkDevices(DW1000Device *device);
    static void removeNetworkDevices(short index);
    
    //setters
    static void setReplyTime(unsigned long replyDelayTimeUs) { _replyDelayTimeUS=replyDelayTimeUs; };
    static void setResetPeriod(unsigned long resetPeriod) { _resetPeriod=resetPeriod; };
    
    //getters 
    static byte* getCurrentAddress() { return _currentAddress; };
    static byte* getCurrentShortAddress() { return _currentShortAddress; };
    static short getNetworkDevicesNumber() { return _networkDevicesNumber; };
    
    //ranging functions
    static short detectMessageType(byte datas[]);
    static void dwloop();
    
    //Handlers:
    static void attachNewRange(void (*handleNewRange)(void)) { _handleNewRange = handleNewRange; };
    static void attachMeasureComplete(void (*handleMeasureComplete)(void)) { _handleMeasureComplete = handleMeasureComplete; };
    
    static float getNearestRange();
    DW1000Device* getDeviceAtIdx(unsigned int idx);
    static DW1000Device* getDistantDevice();
    static DW1000Device* searchDistantDevice(byte shortAddress[]);
    
    //FOR DEBUGGING
    static void visualizeDatas(byte datas[]);
    
  private:
    //other devices in the network
    static DW1000Device _networkDevices[MAX_DEVICES];
    static unsigned short _networkDevicesNumber;
    static unsigned short _lastDistantDevice;
    static byte _currentAddress[8];
    static byte _currentShortAddress[2];
    static byte _lastSentToShortAddress[2]; 
    static DW1000Mac _globalMac;
    static unsigned long timer;
    static short counterForBlink;
    static unsigned long lastForeignPacket;
    
    //Handlers:
    static void (*_handleNewRange)(void);
    static void (*_handleMeasureComplete)(void);
    
    //sketch type (tag or anchor)
    static int _type; //0 for tag and 1 for anchor
    // message flow state
    static volatile byte _expectedMsgId;
    // message sent/received state
    static volatile boolean _sentAck;
    static volatile boolean _receivedAck;
    // protocol error state
    static boolean _protocolFailed;
    // reset line to the chip
    static unsigned int _RST;
    static unsigned int _SS;
    static unsigned int _IRQ;
    static unsigned int _WAKE;
    // watchdog and reset period
    static unsigned long _lastActivity;
    static unsigned long _resetPeriod;
    // reply times (same on both sides for symm. ranging)
    static unsigned long _replyDelayTimeUS;
    //timer Tick delay
    static unsigned int _timerDelay;
    // ranging counter (per second)
    static unsigned int _successRangingCount;
    static unsigned long _rangingCountPeriod;
    
    //_bias correction
    static char _bias_RSL[17];
    //17*2=34 bytes in SRAM
    static short _bias_PRF_16[17];
    //17 bytes in SRAM
    static char _bias_PRF_64[17];
    
    //methods
    static void handleSent();
    static void handleReceived();
    static void noteActivity();
    static void resetInactive();
    
    //global functions:
    static void checkForReset();
    static void checkForInactiveDevices();
    static void copyShortAddress(byte address1[],byte address2[]);
    
    //for ranging protocole (ANCHOR)
    static void transmitInit();
    static void transmit(byte datas[]);
    static void transmit(byte datas[], DW1000Time time);
    static void transmitBlink();
    static void transmitRangingInit(DW1000Device *myDistantDevice);
    static void transmitPollAck(DW1000Device *myDistantDevice);
    static void transmitRangeReport(DW1000Device *myDistantDevice);
    static void transmitRangeFailed(DW1000Device *myDistantDevice);
    static void receiver();
    
    //for ranging protocole (TAG)
    static void transmitPoll(DW1000Device *myDistantDevice);
    static void transmitRange(DW1000Device *myDistantDevice);
    
    //methods for range computation
    static void computeRangeAsymmetric(DW1000Device *myDistantDevice, DW1000Time *myTOF);
    
    static void timerTick();
};

extern DW1000RangingClass DW1000Ranging;

#endif  //__IC_DW1000_Ranging_h__
