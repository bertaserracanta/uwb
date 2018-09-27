/*
  Extended DW1000 Ranging example for radino32 DW1000 and radinoL4 DW1000
  Based on Decawave DW1000 example library
  2017 In-Circuit GmbH
  wiki.in-circuit.de
*/
#include "mainTag.h"

#ifdef ARDUINO_RADINO32
  #include <stm32/l1/iwdg.h>
  #include <stm32/l1/rtc.h>
#endif
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include "IC_DW1000_Ranging_local.h"
#include "MMA7660.h"
#include "config_menu_tag.h"

#define NO_MOVE_TIMEOUT 30000   //After the Tag detected no movement for this timespan it enters deepsleep
#define MY_DW1000_MODE DW1000.MODE_IN_CIRCUIT_5 //Channel setting if runntime config is disabled

#define USE_PULLDOWN_USBDETECT 0
#define NO_MEASURE_WHILE_USB 1  //Stop measurements while charging battery

#define ANTENNA_DELAY 0

#define DoPrintOut 0  //Alway print out measuremts on Serial
#define ALWAYS_ON_TEST 0  //Do not enter deep sleep if no movement is detected

#define LEDFLASHTIME 10

#define MY_DEVICE_TYPE     IC_DW_TAG_TAG

#define ENABLE_RUNTIME_CONFIG 1 //Enable configuration menu on Serial

//set tx power in dBm, 0 to 33.5, step 0.5
//overides configured value (myConfig_tag.tx_power)
//Respect you local wireless regulations!
//#define DW_TX_POWER      33.5
//#define DW_TX_POWER      27.5
//#define DW_TX_POWER      22.5
//#define DW_TX_POWER      12.5
//#define DW_TX_POWER      0.0

#define PIN_BATMEAS_EN   A0
#define PIN_BATMEAS      A3
#define PIN_VUSB_DETECT  25

#define PIN_MM7660_INT A1

#define PIN_LED_G 13
#define PIN_LED_Y  5
#define PIN_LED_R A5

#define LEDG_ON   digitalWrite(PIN_LED_G,HIGH)
#define LEDY_ON   digitalWrite(PIN_LED_Y,HIGH)
#define LEDR_ON   digitalWrite(PIN_LED_R,HIGH)
#define LEDG_OFF  digitalWrite(PIN_LED_G,LOW)
#define LEDY_OFF  digitalWrite(PIN_LED_Y,LOW)
#define LEDR_OFF  digitalWrite(PIN_LED_R,LOW)

//Where to printout information
#define SerialOut if(false)Serial
#define SerialOutBegin() do{}while(0)
//#define SerialOut if(Serial)Serial
//#define SerialOutBegin() Serial.begin()
//#define SerialOut Serial1
//#define SerialOutBegin() Serial1.begin(230400)
#define cmdSerial Serial
#define SerialSpeed 115200

uint32_t ledOnTime;
MMA7660 accelemeter;

uint16_t defShortAddress;
bool batteryOK = false;
float curVcc = 0;
bool wirelessActive = true;
volatile bool gotAccelIsr = false;
bool doBatStatusShow = false;
volatile bool doForceMeasurement = false;

uint8_t cmdOutPutFlag=false;
char cmdOutPutBuf[64];

//#define extraBufLen 4096
#define extraBufLen 0
#if extraBufLen!=0
  uint8_t extraSerialBuf[extraBufLen];
#endif

void setup_tag(uint16_t short_address)
{
  pinLow(PIN_LED_G, OUTPUT);
  pinLow(PIN_LED_Y, OUTPUT);
  pinLow(PIN_LED_R, OUTPUT);

  pinMode(PIN_BATMEAS_EN, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_BATMEAS_EN, HIGH);
  pinMode(PIN_BATMEAS, INPUT_ANALOG);
  pinMode(PIN_VUSB_DETECT, INPUT);

#if extraBufLen!=0
  SerialOut.setTxBuf(extraSerialBuf, extraBufLen);
#endif
  SerialOutBegin();

  #if ARDUINO_RADINO32
    iwdg_set_timeout(64, 0x7FF);
  #endif
  #warning Watchdog timeout
  startWatchdog();

  defShortAddress = short_address;
  config_load_tag(defShortAddress);
  
  // Init MMA7660
  Wire.begin();
  accelemeter.init();

  // Setup async cb timer
  setupDw1000DelayedIsr();
  
  LEDG_ON;
  
  // configure MMA7660
  accelemeter.setMode(MMA7660_STAND_BY);  // standby mode: enable access to registers

  accelemeter.setSampleRateAS(AUTO_SLEEP_120); // 120 samples per second
  //accelemeter.setSampleRateAS(AUTO_SLEEP_1); // 1 sample per second

  accelemeter.setTiltFilter(FILT_2);  // Tilt filter - sample count
  //   accelemeter.setupTapDetection(TAP_XDA | TAP_YDA | TAP_ZDA, 0x0F, 0x0F); // Tap detection on each axis with threshold and debounce count

  //   accelemeter.enableInterrupt(INT_GINT);  // interrupt on every measurement / every sample
  //   accelemeter.enableInterrupt(INT_FBINT);  // Front/Back position change interrupt
  //   accelemeter.enableInterrupt(INT_PLINT);  // Up/Down/Right/Left position change interrupt
  //   accelemeter.enableInterrupt(INT_SHINTX | INT_SHINTY | INT_SHINTZ);  // Shake - Interrupt
  //   accelemeter.enableInterrupt(INT_PDINT);  // Tap - interrupt
  accelemeter.disableAllInterrupts();

  accelemeter.setMode(MMA7660_ACTIVE);  // active mode: measurement activated - no register write possible

  // MMA7660 interrupt pin
  pinMode(PIN_MM7660_INT, INPUT);
  attachInterrupt(PIN_MM7660_INT, callbackFct, FALLING);
  
  LEDY_ON;

  // init DW1000
  DW1000Ranging.initCommunication();
  LEDR_ON;
  DW1000Ranging.wakeUp();
  delay(25);
  LEDG_OFF;
  LEDY_OFF;
  LEDR_OFF;

  // start in low power mode
  accelemeter.setMode(MMA7660_STAND_BY);
  DW1000Ranging.goToDeepSleep(false, false);
  wirelessActive = false;
  batteryOK = false;
  
  ledOnTime = millis();
  SerialOut.println(" init");
}

void loop_tag()
{
#define MAX_ACC_DELTA_WAKEUP  4
#define MAX_ACC_DELTA_ALIVE  2
#define AVG_COUNT     16
#define MOVE_DETECT_INTERVAL 500
  static uint32_t lastAccelMeas = 0;
  static uint32_t lastAccelDetect = 0;
  static int8_t lxa[AVG_COUNT] = {0,}, lya[AVG_COUNT] = {0,}, lza[AVG_COUNT] = {0,};
  static uint8_t lxi = 0, lyi = 0, lzi = 0;
  static uint8_t accDetectCount = 0xFF;
  static float avgBattery = 0;
  
  reloadWatchdog();
  
  if (wirelessActive) DW1000Ranging.dwloop();
  
  if (!doBatStatusShow && ((millis() - ledOnTime) > LEDFLASHTIME))
  {
    LEDG_OFF;
    LEDY_OFF;
    LEDR_OFF;
  }
  
#define BAT_MEAS_COUNT 8
#define BAT_MEAS_INTERVAL 500
  static uint32_t lastBatMeas = 0;
  static float batMeasures[BAT_MEAS_COUNT] = {0, };
  static uint8_t batMeasIdx = 0;
  if (lastBatMeas == 0)
  {
    // initalise battery measurement
    digitalWrite(PIN_BATMEAS_EN, LOW);
    delay(1);
    lastBatMeas = millis();
    avgBattery = 0;
    curVcc = ADC_measVDDA();
    for (uint8_t i = 0 ; i < BAT_MEAS_COUNT ; i++ )
    {
      batMeasures[i] = analogRead(PIN_BATMEAS);
      avgBattery += batMeasures[i];
    }
    digitalWrite(PIN_BATMEAS_EN, HIGH);
    avgBattery /= BAT_MEAS_COUNT;
    avgBattery *= 2*(curVcc/1024);
  }
  if ((millis() - lastBatMeas) > BAT_MEAS_INTERVAL)
  {
    lastBatMeas = millis();
    digitalWrite(PIN_BATMEAS_EN, LOW);
    delay(1);
    curVcc = ADC_measVDDA();
    float batTemp = analogRead(PIN_BATMEAS);
    digitalWrite(PIN_BATMEAS_EN, HIGH);
    batMeasures[batMeasIdx++] = batTemp;
    batMeasIdx %= BAT_MEAS_COUNT;
    avgBattery = 0;
    for (uint8_t i = 0 ; i < BAT_MEAS_COUNT ; i++ )
    {
      avgBattery += batMeasures[i];
    }
    avgBattery /= BAT_MEAS_COUNT;
    avgBattery *= 2*(curVcc/1024);
    DW1000Ranging.batLevel = ((avgBattery-2)*100);
  }

  static bool weHaveUsb = false;
  static uint32_t usbStarted = 0;
#if USE_PULLDOWN_USBDETECT
  static uint32_t lastUsbDetect = 0;
  if ((millis()-lastUsbDetect)>250)
  {
    pinMode(PIN_VUSB_DETECT, INPUT_PULLDOWN);
    delayMicroseconds(250);
    weHaveUsb = !digitalRead(PIN_VUSB_DETECT);
    pinMode(PIN_VUSB_DETECT, INPUT);
  }
#else
  weHaveUsb = digitalRead(PIN_VUSB_DETECT);
#endif
  
#if ENABLE_RUNTIME_CONFIG
  if (usbStarted!=0)
  {
    if (weHaveUsb)
    {
      checkCommandMenu_tag();
    } else {
      SerialOut.println("USB disconnected");
      cmdSerial.end();
      doForceMeasurement = false;
      usbStarted = false;
    }
  }
  else if (weHaveUsb)
  {
    SerialOut.println("start USB");
    cmdSerial.begin(SerialSpeed);
    usbStarted = true;
  }
#endif

  static uint32_t lastBatSignal = 0;
  if (!batteryOK)
  {
    if (!doForceMeasurement && NO_MEASURE_WHILE_USB && weHaveUsb)
    {
      if ((millis()-lastBatSignal)<2000) return;
      if (avgBattery>4.125) { LEDG_ON; delay(250); }
      else if (avgBattery>3.8) { LEDY_ON; delay(250); }
      else { LEDR_ON; delay(125); }
      ledOnTime = millis();
      lastBatSignal = millis();
      return;
    }
    if (avgBattery > 3.20)
    {
      SerialOut.println("Bat becomes OK");
      accelemeter.setMode(MMA7660_STAND_BY);
      gotAccelIsr = false;
      accelemeter.setSampleRateAS(AUTO_SLEEP_120);
      accelemeter.enableInterrupt(INT_GINT);
      accelemeter.setMode(MMA7660_ACTIVE);
      while(!gotAccelIsr){}
      accelemeter.disableAllInterrupts();
      batteryOK = true;
      lastAccelMeas = 0;
      return;
    }

    // battery very low. Wait for charge.
    sleepForMs(2500);
    lastBatMeas = millis() - (BAT_MEAS_INTERVAL + 1);
    return;
  }
  else if ((!doForceMeasurement && NO_MEASURE_WHILE_USB && weHaveUsb) || avgBattery<3.00)
  {
    if (!doForceMeasurement && NO_MEASURE_WHILE_USB && weHaveUsb)
      SerialOut.println("V USB detected");
    else
      SerialOut.println("Bat becomes BAD");
    batteryOK = false;
    LEDG_OFF;
    LEDY_OFF;
    LEDR_OFF;
    DW1000Ranging.goToDeepSleep(false, false);
    wirelessActive = false;
    accelemeter.setMode(MMA7660_STAND_BY);
    lastBatSignal = millis();
    return;
  }
  
  if (lastAccelMeas == 0)
  {
    int8_t x, y, z;
    lastAccelDetect = millis();
    lastAccelMeas = millis();
    accDetectCount = 0;
    accelemeter.getXYZ(&x, &y, &z);
    for (uint8_t i = 0 ; i < AVG_COUNT ; i++ )
    {
      lxa[i] = x; lya[i] = y; lza[i] = z;
    }
  }

  if ((millis() - lastAccelMeas) > MOVE_DETECT_INTERVAL)
  {
    int8_t x, y, z;
    float lx = 0, ly = 0, lz = 0;
    lastAccelMeas = millis();
    accelemeter.getXYZ(&x, &y, &z);

    for (uint8_t i = 0 ; i < AVG_COUNT ; i++ )
    {
      lx += lxa[i]; ly += lya[i]; lz += lza[i];
    }
    lx /= AVG_COUNT; ly /= AVG_COUNT; lz /= AVG_COUNT;
    lxa[lxi++] = x; lya[lyi++] = y; lza[lzi++] = z;
    lxi %= AVG_COUNT; lyi %= AVG_COUNT; lzi %= AVG_COUNT;

#if ALWAYS_ON_TEST
    if (true)
#else
    if (
      doForceMeasurement ||
      (
        ((millis() - lastAccelDetect) > NO_MOVE_TIMEOUT)
        && ( x > lx + MAX_ACC_DELTA_WAKEUP || x < lx - MAX_ACC_DELTA_WAKEUP || y > ly + MAX_ACC_DELTA_WAKEUP || y < ly - MAX_ACC_DELTA_WAKEUP || z > lz + MAX_ACC_DELTA_WAKEUP || z < lz - MAX_ACC_DELTA_WAKEUP )
      ) || (
        !((millis() - lastAccelDetect) > NO_MOVE_TIMEOUT)
        && ( x > lx + MAX_ACC_DELTA_ALIVE || x < lx - MAX_ACC_DELTA_ALIVE || y > ly + MAX_ACC_DELTA_ALIVE || y < ly - MAX_ACC_DELTA_ALIVE || z > lz + MAX_ACC_DELTA_ALIVE || z < lz - MAX_ACC_DELTA_ALIVE )
      )
    )
#endif
    {
      lastAccelDetect = millis();

      if (accDetectCount > 4) accDetectCount = 4;
      if (++accDetectCount == 4)
      {
        SerialOut.println("DW1000 start");
        wirelessActive = true;
        //init DW1000
        DW1000Ranging.wakeUp();
        DW1000Ranging.initCommunication();
        DW1000Ranging.attachNewRange(newRange_tag);
        DW1000Ranging.attachRequestDelayedIsr(dw1000RequestDelayedIsr);
        //anchor means tag due to library naming conditions
#if ENABLE_RUNTIME_CONFIG
        switch(myConfig_tag.channel)
        {
          default:
          case 1: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
          case 2: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
          case 3: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
          case 4: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
          case 5: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
          case 7: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
        }
        #ifdef DW_TX_POWER
          myConfig_tag.tx_power = (DW_TX_POWER*10);
        #endif
        DW1000.setManualTxPower(((float)myConfig_tag.tx_power)/10);
#else
        DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", MY_DW1000_MODE, defShortAddress, MY_DEVICE_TYPE);
        #ifdef DW_TX_POWER
          DW1000.setManualTxPower(DW_TX_POWER);
        #endif
#endif
        DW1000.setAntennaDelay(ANTENNA_DELAY);
        ledOnTime = millis();
      }
    }
    else
    {
      if (accDetectCount < 5) accDetectCount = 0;
    }

    uint8_t tiltStatus = accelemeter.getTiltStatus();
    doBatStatusShow=false;
    #if 1
      switch (accelemeter.getOrientationStatus(tiltStatus))
      {
        case POLA_UP: doBatStatusShow=true; break;
        default: break;
      }
    #else
      switch (accelemeter.getOrientationStatus(tiltStatus))
      {
        case POLA_UP:       SerialOut.println("UP"); doBatStatusShow=true; break;
        case POLA_DOWN:     SerialOut.println("DOWN");  break;
        case POLA_LEFT:     SerialOut.println("LEFT");  break;
        case POLA_RIGHT:    SerialOut.println("RIGHT");  break;
        case POLA_UNKNOWN:  SerialOut.println("NONE");  break;
        default:            SerialOut.println("ERROR");  break;
      }
    #endif

    if ((millis() - lastAccelDetect) > NO_MOVE_TIMEOUT)
    {
      LEDG_OFF;
      LEDY_OFF;
      LEDR_OFF;
      if (wirelessActive)
        DW1000Ranging.goToDeepSleep(false, false);
      wirelessActive = false;
      accelemeter.setMode(MMA7660_STAND_BY);
      sleepForMs(1000);
      accelemeter.setMode(MMA7660_STAND_BY);
      accelemeter.setSampleRateAS(AUTO_SLEEP_120); // 120 samples per second
      accelemeter.setMode(MMA7660_ACTIVE);

      accDetectCount = 0;
      lastAccelMeas = millis() - (MOVE_DETECT_INTERVAL - 100);
      lastAccelDetect = millis() - NO_MOVE_TIMEOUT;
      lastBatMeas = millis() - (BAT_MEAS_INTERVAL - 50);
    }
    else if (doBatStatusShow)
    {
      static bool altToggler = false;
      if ((altToggler=!altToggler))
      {
        LEDG_OFF;
        LEDY_OFF;
        LEDR_OFF;
        ledOnTime = millis();
               if (avgBattery>4.00) {
          LEDG_ON;
        } else if (avgBattery>3.75) {
          LEDG_ON;
          LEDY_ON;
        } else if (avgBattery>3.50) {
          LEDY_ON;
        } else if (avgBattery>3.25) {
          LEDY_ON;
          LEDR_ON;
        } else {
          LEDR_ON;
        }
      }
    }
  }
}

void sleepForMs(uint32_t duration)
{
  SerialOut.flush();
  //Here you may add code to put the stm32 to stop mode (<2uA consumption)
  //doStopModeForMs(duration);
  //If you need help implementing that procedure you may
  //contact us at support@in-circuit.de
  delay(duration);
  return;
}

// MMA7660 interrupt callback-function
void callbackFct()
{
  gotAccelIsr = true;
}

// DW1000 single measure callback
void newRange_tag()
{
  if (!doBatStatusShow)
  {
    if(DW1000Ranging.getDistantDevice()->getRXPower() > -75)
    {
      LEDG_ON;
    }
    else if(DW1000Ranging.getDistantDevice()->getRXPower() > -90)
    {
      LEDY_ON;
    }
    else
    {
      LEDR_ON;
    }
    ledOnTime = millis();
  }
  
  if (doForceMeasurement && !cmdOutPutFlag)
  {
    snprintf(cmdOutPutBuf,sizeof(cmdOutPutBuf),"%x\t%.2fm\t%.2fdBm\r\n",
      DW1000Ranging.getDistantDevice()->getShortAddress(),
      DW1000Ranging.getDistantDevice()->getRange(),
      DW1000Ranging.getDistantDevice()->getRXPower()
      );
    cmdOutPutFlag = true;
  }

#if DoPrintOut==1
  SerialOut.print(" ");
  SerialOut.print(millis());
  SerialOut.print(" from: ");
  SerialOut.print(DW1000Ranging.getDistantDevice()->getShortAddress(), HEX);
  SerialOut.print("\t Range: ");
  SerialOut.print(DW1000Ranging.getDistantDevice()->getRange());
  SerialOut.print(" m");
  SerialOut.print("\t RX power: ");
  SerialOut.print(DW1000Ranging.getDistantDevice()->getRXPower());
  SerialOut.print(" dBm");
  SerialOut.print("\t Our Idx: ");
  SerialOut.print(DW1000Ranging.getDistantDevice()->ourLastPos);
  SerialOut.print("/");
  SerialOut.print(DW1000Ranging.getDistantDevice()->knownCount);
  SerialOut.println();
#endif
}

#ifdef ARDUINO_RADINO32
//radino32
extern "C" {
  void tim7_isr(void)
  {
    tim_disable_update_interrupt_on_any(TIM7);
    tim_clear_interrupt(TIM7, TIM_UPDATE);
    tim_disable_counter(TIM7);
    DW1000Ranging.execDelayedIsr();
  }
}
#else
//radinoL4
  void delayedIsrCB()
  {
    DW1000Ranging.execDelayedIsr();
  }
#endif

//1us timer for delayed isr
void setupDw1000DelayedIsr()
{
  #ifdef ARDUINO_RADINO32
    rcc_enable_clock(RCC_TIM7);
    tim_enable_one_pulse_mode(TIM7);
    tim_disable_update_interrupt_on_any(TIM7);
    tim_load_prescaler_value(TIM7, TIMX_CLK_APB1 / 100000 - 1); //Prescaler to 100KHz
  #else
    timer_init();
    timer_attachCallback(delayedIsrCB);
  #endif
}

//triggers timer isr after delayUs us
void dw1000RequestDelayedIsr(uint32_t delayUs)
{
  #ifdef ARDUINO_RADINO32
    delayUs /= 10;
    tim_set_autoreload_value(TIM7, delayUs - 1);
    tim_enable_interrupt(TIM7, TIM_UPDATE);
    nvic_enable_irq(NVIC_TIM7_IRQ);
    tim_clear_interrupt(TIM7, TIM_UPDATE);
    tim_enable_counter(TIM7);
  #else
    timer_RequestISR(delayUs);
  #endif
}

void checkCommandMenu_tag()
{
  static unsigned long cmdMenuOutput = 0;  //was the information line printed
  static unsigned char cmdBuf[5];  //buffer for incoming chars
  static unsigned char cmdBufPos = 0;  //where in the buffer to store next character
  unsigned char curChar = '\0';
  unsigned short * valToModify = NULL;
  bool cmdOk = false;

  if (!cmdSerial) return;
  while (cmdSerial.available())
  {
    curChar = cmdSerial.read();
    if (curChar != '\r' && curChar != '\n')
    {
      //Shift char into buf
      if (cmdBufPos >= 5)
      {
        cmdBuf[0] = cmdBuf[1];
        cmdBuf[1] = cmdBuf[2];
        cmdBuf[2] = cmdBuf[3];
        cmdBuf[3] = cmdBuf[4];
        cmdBuf[4] = curChar;
      }
      else
      {
        cmdBuf[cmdBufPos++] = curChar;
      }
    }
    else
    {
      //We got a line ending -> react
      //Trash remaining chars (e.g. Windows \r\n)
      delay(5);
      while (cmdSerial.available()) cmdSerial.read();

      //print out current buf content
      if (cmdBufPos>0) cmdSerial.write(cmdBuf[0]);
      if (cmdBufPos>1) cmdSerial.write(cmdBuf[1]);
      if (cmdBufPos>2) cmdSerial.write(cmdBuf[2]);
      if (cmdBufPos>3) cmdSerial.write(cmdBuf[3]);
      if (cmdBufPos>4) cmdSerial.write(cmdBuf[4]);
      cmdSerial.println("   ");
      if (cmdBufPos>0) cmdSerial.write(cmdBuf[0]);
      else cmdBuf[0]=0;
      cmdSerial.print(": ");
      valToModify = NULL;
      cmdOk = false;

      //Start command handling
      switch (cmdBuf[0])
      {
        case'f': case'F': cmdSerial.print("Toggle forced measurement"); cmdOk = true; doForceMeasurement=!doForceMeasurement; break;
        case'r': case'R': cmdSerial.print("Reload config from EEPROM"); cmdOk = true; config_load_tag(defShortAddress); break;
        case'c': case'C': cmdSerial.print("Set Channel"); valToModify = &myConfig_tag.channel; break;
        case'x': case'X': cmdSerial.print("Set TX strength"); valToModify = &myConfig_tag.tx_power; break;
        case'a': case'A': cmdSerial.print("Set Address"); valToModify = &myConfig_tag.ourAddr; break;
        case's': case'S': cmdSerial.print("Storing config to EEPROM"); cmdOk = true; config_store_tag(); break;
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
          myConfigTag_t confBack = myConfig_tag;
          cmdSerial.write(cmdBuf[1]);
          if (cmdBufPos > 2) cmdSerial.write(cmdBuf[2]);
          if (cmdBufPos > 3) cmdSerial.write(cmdBuf[3]);
          if (cmdBufPos > 4) cmdSerial.write(cmdBuf[4]);
          cmdSerial.print(": ");
          if ( (valToModify!=&myConfig_tag.ourAddr)
            && ( !isdigit(cmdBuf[1]) || (cmdBufPos > 2 && !isdigit(cmdBuf[2])) || (cmdBufPos > 3 && !isdigit(cmdBuf[3])) || (cmdBufPos > 4 && !isdigit(cmdBuf[4])) )
            )
          {
            cmdSerial.print("Invalid value");  //got non digit param
          } else if ( !isxdigit(cmdBuf[1]) || (cmdBufPos > 2 && !isxdigit(cmdBuf[2])) || (cmdBufPos > 3 && !isxdigit(cmdBuf[3])) || (cmdBufPos > 4 && !isxdigit(cmdBuf[4])) )
          {
            cmdSerial.print("Invalid value");  //got non hex param
          }
          else
          {
            if (valToModify!=&myConfig_tag.ourAddr)
            {
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
              if (cmdBufPos > 4)
              {
                *valToModify *= 10;
                *valToModify += fromdigitch(cmdBuf[4]);
              }
            }
            else
            {
              *valToModify = fromhexch(cmdBuf[1]);
              if (cmdBufPos > 2)
              {
                *valToModify <<= 4;
                *valToModify += fromhexch(cmdBuf[2]);
              }
              if (cmdBufPos > 3)
              {
                *valToModify <<= 4;
                *valToModify += fromhexch(cmdBuf[3]);
              }
              if (cmdBufPos > 4)
              {
                *valToModify <<= 4;
                *valToModify += fromhexch(cmdBuf[4]);
              }
            }
            
            if (myConfig_tag.channel<1) myConfig_tag.channel=5;
            if (myConfig_tag.channel==6) myConfig_tag.channel=5;
            if (myConfig_tag.channel>7) myConfig_tag.channel=5;
            if (myConfig_tag.tx_power>335) myConfig_tag.tx_power=335;
            myConfig_tag.tx_power /= 5;
            myConfig_tag.tx_power *= 5;
            cmdSerial.print("New value: ");
            if (valToModify!=&myConfig_tag.ourAddr)
              cmdSerial.print(*valToModify);
            else
              cmdSerial.print(*valToModify,HEX);
            cmdOk = true;
            
            if (wirelessActive)
            {
              if ((myConfig_tag.channel != confBack.channel) || (myConfig_tag.tx_power != confBack.tx_power))
              {
                DW1000Ranging.goToDeepSleep(false, false);
                delay(50);
                DW1000Ranging.wakeUp();
                DW1000Ranging.initCommunication();
                DW1000Ranging.attachNewRange(newRange_tag);
                DW1000Ranging.attachRequestDelayedIsr(dw1000RequestDelayedIsr);
                switch(myConfig_tag.channel)
                {
                  default:
                  case 1: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_1, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                  case 2: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_2, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                  case 3: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_3, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                  case 4: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_4, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                  case 5: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_5, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                  case 7: DW1000Ranging.startAsAnchor("7D:00:22:EA:82:60:3B:9C", DW1000.MODE_IN_CIRCUIT_7, myConfig_tag.ourAddr, MY_DEVICE_TYPE); break;
                }
                DW1000.setManualTxPower(((float)myConfig_tag.tx_power)/10);
              }
            }
          }
        }
        cmdSerial.println("   ");
      }
      if (cmdOk)  //command handled successfully
        cmdSerial.println("OK");
      else  //error occured during command handling
        cmdSerial.println("ERR");
      cmdMenuOutput = cmdBuf[0] = cmdBuf[1] = cmdBuf[2] = cmdBuf[3] = cmdBuf[4] = cmdBufPos = 0;
    }
  }
  if (cmdMenuOutput == 0) //a printout of the info line was scheduled
  {
    cmdMenuOutput = 1;
    cmdSerial.print("A)ddr:0x");
    cmdSerial.print(((myConfig_tag.ourAddr>>12)&0xF),HEX);
    cmdSerial.print(((myConfig_tag.ourAddr>> 8)&0xF),HEX);
    cmdSerial.print(((myConfig_tag.ourAddr>> 4)&0xF),HEX);
    cmdSerial.print(((myConfig_tag.ourAddr>> 0)&0xF),HEX);
    cmdSerial.print("  C)han:");
    cmdSerial.print(myConfig_tag.channel);
    cmdSerial.print("  X)Str:");
    cmdSerial.print(myConfig_tag.tx_power / 10); cmdSerial.print("."); cmdSerial.print(myConfig_tag.tx_power % 10);
    cmdSerial.print("  R)eload");
    cmdSerial.print("  S)tore");
    cmdSerial.print("  F)orceMeas:");
    cmdSerial.print(doForceMeasurement?"Y":"N");
    cmdSerial.print("\r\n> ");
  }
  if (cmdOutPutFlag)
  {
    cmdSerial.print(cmdOutPutBuf);
    cmdOutPutFlag = false;
  }
}
