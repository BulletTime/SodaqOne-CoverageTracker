/*
  The MIT License (MIT)

  Copyright © 2018 Sven Agneessens <sven.agneessens@gmail.com>

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
  THE SOFTWARE.
*/

#include <Arduino.h>
#include <Wire.h>
#include "Enums.h"
#include "LedColor.h"
#include "LoRaHelper.h"
#include "MyData.h"
#include "MyTime.h"
#include "RTCTimer.h"
#include "RTCZero.h"
#include "Sodaq_LIS3DE.h"
#include "Sodaq_wdt.h"
#include "Structs.h"
#include "ublox.h"
#include "Utils.h"

#define DEBUG false
#define DEBUG_LORA false

#define RESEARCH true
#define ENABLE_LED true

#define VERSION "0.2.2"
#define PROJECT_NAME "LoRa Coverage Logger"
#define STARTUP_DELAY 5000

#define SETTINGS_DEFAULT_TIMEOUT 10 // 10 seconds

#define GPS true
#define GPS_DEFAULT_TIMEOUT 30 // 15 seconds
#define GPS_COMM_CHECK_TIMEOUT 3 // 3 seconds
#define GPS_TIME_VALIDITY 0b00000011 // date and time (but not fully resolved)
#define GPS_FIX_FLAGS 0b00000001 // just gnssFixOK
#define GPS_MIN_SAT_COUNT 4

#define MAX_RTC_EPOCH_OFFSET 25

#define ACCELEROMETER true
#define DEFAULT_MOVEMENT_PERCENTAGE 6
#define DEFAULT_MOVEMENT_DURATION 50 // d = x * 1/(10 Hz) MAX: 127

#define DEFAULT_TEMPERATURE_SENSOR_OFFSET 33
#define DEFAULT_LORA_PORT 2
#define DEFAULT_IS_OTAA_ENABLED 1
#define DEFAULT_DEVADDR_OR_DEVEUI "0000000000000000"
#define DEFAULT_APPSKEY_OR_APPEUI "0000000000000000"
#define DEFAULT_NWSKEY_OR_APPKEY "00000000000000000000000000000000"
#define DEFAULT_ADR_ON 0
#define DEFAULT_ACK_ON 0
#define DEFAULT_RECONNECT_ON_TRANSMISSION 1
#define DEFAULT_REPEAT_TRANSMISSION_COUNT 0
// #define DEFAULT_SPREADING_FACTOR 7
#define DEFAULT_POWER 1

#define DEBUG_STREAM SerialUSB
#define CONSOLE_STREAM SerialUSB
#define LORA_STREAM Serial1
#define BAUDRATE 115200

#define consolePrint(x) CONSOLE_STREAM.print(x)
#define consolePrintln(x) CONSOLE_STREAM.println(x)

#define debugPrint(x) if (DEBUG) { DEBUG_STREAM.print(x); }
#define debugPrintln(x) if (DEBUG) { DEBUG_STREAM.println(x); }

#ifndef LORA_RESET
#define LORA_RESET -1
#endif

struct GPSData gpsData;

RTCZero rtc;
RTCTimer timer;
Sodaq_LIS3DE accelerometer;
Time time;
UBlox ublox;

bool isGpsDataNew;

volatile bool minuteFlag;
volatile bool acceleration;
volatile bool accelerationFlag;

static uint8_t lastResetCause;
static bool isRtcInitialized;
static bool isGpsInitialized;
static bool isAccelerometerInitialized;
static bool isDeviceInitialized;
static int64_t rtcEpochDelta;

static uint8_t loraHWEui[8];
static bool isLoraHWEuiInitialized;

static SF sfState = SF12;

void setup();
void loop();

uint32_t getNow();
void setNow(uint32_t now);
void initRtc();
void rtcAlarmHandler();
void initRtcTimer();
void resetRtcTimerEvents();
void setupBOD33();
void initSleep();
bool initGps();
void setGpsActive(bool on);
void initAccelerometer();
void accelerometerInt1Handler();
bool initLora(LoraInitConsoleMessages messages, LoraInitJoin join);
void systemSleep();
void runLoraModuleSleepExtendEvent(uint32_t now);
void runResearchEvent(uint32_t now);
void runAccelerometerEvent(uint32_t now);
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt);
bool getGpsFix(uint32_t timeout);
bool getGpsCoordinates();
bool transmitLastData();

static void printCpuResetCause(Stream& stream);
static void printBootUpMessage(Stream& stream);
static void printMyDataMessage(Stream& stream);

void setup() {
  // Allow power to remain on
  pinMode(ENABLE_PIN_IO, OUTPUT);
  digitalWrite(ENABLE_PIN_IO, HIGH);

  lastResetCause = PM->RCAUSE.reg;

  // In case of reset (this is probably unnecessary)
  sodaq_wdt_disable();

  // Setup brown-out detection
  setupBOD33();

  sodaq_wdt_enable(WDT_PERIOD_4X);
  sodaq_wdt_reset();

  CONSOLE_STREAM.begin(BAUDRATE);
  if ((long)&CONSOLE_STREAM != (long)&DEBUG_STREAM) {
    DEBUG_STREAM.begin(BAUDRATE);
  }

  setLedColor(RED);
  sodaq_wdt_safe_delay(STARTUP_DELAY);
  printBootUpMessage(CONSOLE_STREAM);

  initSleep();
  initRtc();

  Wire.begin();

  // init myData
  myData.read();
  printMyDataMessage(CONSOLE_STREAM);
  myData.setPower(DEFAULT_POWER);
  myData.commit();

  // if allowed, init early for faster initial fix
  if (GPS) {
    isGpsInitialized = initGps();
  }

  initLora(LORA_INIT_SHOW_CONSOLE_MESSAGES, LORA_INIT_JOIN);

  if (ACCELEROMETER) {
    initAccelerometer();
    isAccelerometerInitialized = true;
  }

  initRtcTimer();

  isDeviceInitialized = true;

  consolePrintln("** Boot-up completed successfully!");
  sodaq_wdt_reset();

  // disable the USB if not needed for debugging
  if (!DEBUG || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
    consolePrintln("The USB is going to be disabled now.");
    debugPrintln("The USB is going to be disabled now.");

    SerialUSB.flush();
    sodaq_wdt_safe_delay(3000);
    SerialUSB.end();
    USBDevice.detach();
    USB->DEVICE.CTRLA.reg &= ~USB_CTRLA_ENABLE; // Disable USB
  }

  // disable the debug stream if it is not disabled by the above
  if (!DEBUG && ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
    DEBUG_STREAM.flush();
    DEBUG_STREAM.end();
  }

  // disable the console stream if it is not disabled by the above
  if ((long)&CONSOLE_STREAM != (long)&SerialUSB) {
    CONSOLE_STREAM.flush();
    CONSOLE_STREAM.end();
  }

  if (GPS) {
    if (getGpsFix(300)) {
      setLedColor(GREEN);
      sodaq_wdt_safe_delay(1500);
    }
  } else {
    setLedColor(GREEN);
    sodaq_wdt_safe_delay(1500);
  }

  // resetRtcTimerEvents();
}

/**
   Returns the current datetime (seconds since unix epoch).
*/
uint32_t getNow() {
  return rtc.getEpoch();
}

/**
   Sets the RTC epoch and "rtcEpochDelta".
*/
void setNow(uint32_t newEpoch) {
  uint32_t currentEpoch = getNow();

  debugPrint("Setting RTC from ");
  debugPrint(currentEpoch);
  debugPrint(" to ");
  debugPrintln(newEpoch);

  rtcEpochDelta = newEpoch - currentEpoch;
  rtc.setEpoch(newEpoch);

  timer.adjust(currentEpoch, newEpoch);

  isRtcInitialized = true;
}

/**
   Initializes the RTC.
*/
void initRtc() {
  rtc.begin();

  // Schedule the wakeup interrupt for every minute
  // Alarm is triggered 1 cycle after match
  rtc.setAlarmSeconds(59);
  rtc.enableAlarm(RTCZero::MATCH_SS); // alarm every minute

  // Attach handler
  rtc.attachInterrupt(rtcAlarmHandler);

  // This sets it to 2000-01-01
  rtc.setEpoch(0);
}

/**
   Runs every minute by the rtc alarm.
*/
void rtcAlarmHandler() {
  minuteFlag = true;
}

/**
   Initializes the RTC Timer and schedules the default events.
*/
void initRtcTimer() {
  timer.setNowCallback(getNow); // set how to get the current time
  timer.allowMultipleEvents();

  resetRtcTimerEvents();
}

/**
   Clears the RTC Timer events and schedules the default events.
*/
void resetRtcTimerEvents() {
  timer.clearAllEvents();

  if (RESEARCH) {
    timer.every(30, runResearchEvent);
  } else {
    timer.every(5 * 60, runAccelerometerEvent);
  }

  // if lora is not enabled, schedule an event that takes care of extending the sleep time of the module
  if (!LoRa.isInitialized()) {
    timer.every(24 * 60 * 60, runLoraModuleSleepExtendEvent); // once a day
  }
}

/**
   Setup BOD33

   Setup BOD the way we want it.
    - BOD33USERLEVEL = 0x30 - shutdown at 3.07 Volt
    - BOD33_EN = [X] //Enabled
    - BOD33_ACTION = 0x01
    - BOD33_HYST = [X] //Enabled
*/
void setupBOD33() {
  SYSCTRL->BOD33.bit.LEVEL = 0x30;    // 3.07 Volt
  SYSCTRL->BOD33.bit.ACTION = 1;      // Go to Reset
  SYSCTRL->BOD33.bit.ENABLE = 1;      // Enabled
  SYSCTRL->BOD33.bit.HYST = 1;        // Hysteresis on
  while (!SYSCTRL->PCLKSR.bit.B33SRDY) {
    /* Wait for synchronization */
  }
}

/**
  Initializes the CPU sleep mode.
*/
void initSleep() {
  // Set the sleep mode
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
}

/**
   Initializes the GPS and leaves it on if succesful.
   Returns true if successful.
*/
bool initGps() {
  pinMode(GPS_ENABLE, OUTPUT);
  pinMode(GPS_TIMEPULSE, INPUT);

  // attempt to turn on and communicate with the GPS
  ublox.enable();
  ublox.flush();

  uint32_t startTime = getNow();
  bool found = false;
  while (!found && (getNow() - startTime <= GPS_COMM_CHECK_TIMEOUT)) {
    sodaq_wdt_reset();

    found = ublox.exists();
  }

  // check for success
  if (found) {
    setGpsActive(true); // properly turn on before returning

    return true;
  }

  consolePrintln("*** GPS not found!");
  debugPrintln("*** GPS not found!");

  // turn off before returning in case of failure
  setGpsActive(false);

  return false;
}

/**
 * Turns the GPS on or off.
 */
void setGpsActive(bool on) {
  sodaq_wdt_reset();

  if (on) {
    ublox.enable();
    ublox.flush();

    sodaq_wdt_safe_delay(80);

    PortConfigurationDDC pcd;

    uint8_t maxRetries = 6;
    int8_t retriesLeft;

    retriesLeft = maxRetries;
    while (!ublox.getPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
      debugPrintln("Retrying ublox.getPortConfigurationDDC(&pcd)...");
      sodaq_wdt_safe_delay(15);
    }
    if (retriesLeft == -1) {
      debugPrintln("ublox.getPortConfigurationDDC(&pcd) failed!");

      return;
    }

    pcd.outProtoMask = 1; // Disable NMEA
    retriesLeft = maxRetries;
    while (!ublox.setPortConfigurationDDC(&pcd) && (retriesLeft-- > 0)) {
      debugPrintln("Retrying ublox.setPortConfigurationDDC(&pcd)...");
      sodaq_wdt_safe_delay(15);
    }
    if (retriesLeft == -1) {
      debugPrintln("ublox.setPortConfigurationDDC(&pcd) failed!");

      return;
    }

    ublox.CfgMsg(UBX_NAV_PVT, 1); // Navigation Position Velocity TimeSolution
    ublox.funcNavPvt = delegateNavPvt;
  } else {
    ublox.disable();
  }
}

/**
  Initializes the accelerometer functionality (interrupt on acceleration).
*/
void initAccelerometer() {
  pinMode(ACCEL_INT1, INPUT);
  attachInterrupt(ACCEL_INT1, accelerometerInt1Handler, CHANGE);

  // Configure EIC to use GCLK1 which uses XOSC32K, XOSC32K is already running in standby
  // This has to be done after the first call to attachInterrupt()
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(GCM_EIC) |
                      GCLK_CLKCTRL_GEN_GCLK1 |
                      GCLK_CLKCTRL_CLKEN;

  accelerometer.enable(false,
                       Sodaq_LIS3DE::NormalLowPower10Hz,
                       Sodaq_LIS3DE::XYZ,
                       Sodaq_LIS3DE::Scale2g,
                       false);
  // Turn on high-pass filter
  accelerometer.enableHighPassFilter();

  sodaq_wdt_safe_delay(100);

  accelerometer.enableInterrupt1(
    Sodaq_LIS3DE::XLow | Sodaq_LIS3DE::YLow | Sodaq_LIS3DE::ZLow,
    DEFAULT_MOVEMENT_PERCENTAGE * 2.0 / 100.0,
    DEFAULT_MOVEMENT_DURATION,
    Sodaq_LIS3DE::AndCombination);
}

/**
   Runs every time acceleration is over the limits
   set by the user (if enabled).
*/
void accelerometerInt1Handler() {
  acceleration = !digitalRead(ACCEL_INT1);
  accelerationFlag = true;
}

/**
    Initializes the lora module according to the given operation (join or skip).
    Returns true if the operation was successful.
*/
bool initLora(LoraInitConsoleMessages messages, LoraInitJoin join) {
  debugPrintln("Initializing LoRa...");

  if (messages == LORA_INIT_SHOW_CONSOLE_MESSAGES) {
    consolePrintln("Initializing LoRa...");
  }

  if (DEBUG_LORA) {
    LoRaBee.setDiag(DEBUG_STREAM);
    LoRa.setDiag(DEBUG_STREAM);
  }

  bool result;

  LORA_STREAM.begin(LoRaBee.getDefaultBaudRate());
  result = LoRaBee.init(LORA_STREAM, LORA_RESET);
  LoRa.init(LoRaBee, getNow);

  // set keys and parameters
  LoRa.setKeys(DEFAULT_DEVADDR_OR_DEVEUI, DEFAULT_APPSKEY_OR_APPEUI, DEFAULT_NWSKEY_OR_APPKEY);
  LoRa.setOtaaOn(DEFAULT_IS_OTAA_ENABLED);
  LoRa.setAdrOn(DEFAULT_ADR_ON);
  LoRa.setAckOn(DEFAULT_ACK_ON);
  LoRa.setReconnectOnTransmissionOn(DEFAULT_RECONNECT_ON_TRANSMISSION);
  LoRa.setDefaultLoRaPort(DEFAULT_LORA_PORT);
  LoRa.setRepeatTransmissionCount(DEFAULT_REPEAT_TRANSMISSION_COUNT);
  LoRa.setSpreadingFactor(SF12);
  LoRa.setPowerIndex(DEFAULT_POWER);

  if (join == LORA_INIT_JOIN) {
    result = LoRa.join();

    if (messages == LORA_INIT_SHOW_CONSOLE_MESSAGES) {
      if (result) {
        consolePrintln("LoRa initialized.");
        if (RESEARCH) {
          uint8_t dcycle = 7; // = (100/x) - 1
          LoRaBee.setMacParam("ch dcycle 0 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 1 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 2 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 3 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 4 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 5 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 6 ", dcycle);
          LoRaBee.setMacParam("ch dcycle 7 ", dcycle);
          debugPrintln("Research: every channel [0-7] is set to 12.5%")
        }
      }
      else {
        consolePrintln("LoRa initialization failed!");
      }
    }
  }

  LoRa.setActive(false); // make sure it is off
  return result;
}

void loop() {
  if (sodaq_wdt_flag) {
    // Reset watchdog
    sodaq_wdt_reset();
    sodaq_wdt_flag = false;

    LoRa.loopHandler();
  }

  if (accelerationFlag) {
    if (acceleration) {
      if (ENABLE_LED) {
        setLedColor(MAGENTA);
      }
      sfState = SF12;
      // sodaq_wdt_safe_delay(500);
      debugPrintln("Movement detected .. -> sf reset");
    } else {
      if (ENABLE_LED) {
        setLedColor(YELLOW);
        // sodaq_wdt_safe_delay(500);
      }

      timer.update();
    }

    accelerationFlag = false;
  }

  if (minuteFlag) {
    if (ENABLE_LED) {
        setLedColor(BLUE);
    }

    timer.update(); // handle scheduled events

    minuteFlag = false;
  }

  systemSleep();
}

/**
   Powers down all devices and puts the system to deep sleep.
*/
void systemSleep() {
  LORA_STREAM.flush();

  setLedColor(NONE);
  setGpsActive(false); // explicitly disable after resetting the pins

  // go to sleep, unless USB is used for debugging
  if (!DEBUG || ((long)&DEBUG_STREAM != (long)&SerialUSB)) {
    noInterrupts();
    if (!(sodaq_wdt_flag || minuteFlag)) {
      interrupts();

      __WFI(); // SAMD sleep
    }
    interrupts();
  }
}

/**
   Wakes up the lora module to put it back to sleep, i.e. extends the sleep period
*/
void runLoraModuleSleepExtendEvent(uint32_t now) {
  debugPrintln("Extending LoRa module sleep period.");

  LoRa.extendSleep();
}

void runResearchEvent(uint32_t now) {
  if (!acceleration) {
    if (getGpsCoordinates() && transmitLastData()) {
      while (!acceleration && sfState != SF12) {
        if (!transmitLastData()) {
          sfState = SF12;
          break;
        }
      }
    }
  }
}

void runAccelerometerEvent(uint32_t now) {
  if (!acceleration) {
    if (sfState == SF12) {
      if (getGpsCoordinates()) {
        transmitLastData();
      }
    } else {
      transmitLastData();
    }
  }
}

/**
 *  Checks validity of data, adds valid points to the points list, syncs the RTC
 */
void delegateNavPvt(NavigationPositionVelocityTimeSolution* NavPvt) {
  sodaq_wdt_reset();

  if (!isGpsInitialized) {
    debugPrintln("delegateNavPvt exiting because GPS is not initialized.");

    return;
  }

  // note: db_printf gets enabled/disabled according to the "DEBUG" define (ublox.cpp)
  ublox.db_printf("%4.4d-%2.2d-%2.2d %2.2d:%2.2d:%2.2d.%d valid=%2.2x lat=%d lon=%d sats=%d fixType=%2.2x\r\n",
      NavPvt->year, NavPvt->month, NavPvt->day,
      NavPvt->hour, NavPvt->minute, NavPvt->seconds, NavPvt->nano, NavPvt->valid,
      NavPvt->lat, NavPvt->lon, NavPvt->numSV, NavPvt->fixType);

  // sync the RTC time
  if ((NavPvt->valid & GPS_TIME_VALIDITY) == GPS_TIME_VALIDITY) {
    uint32_t epoch = time.mktime(NavPvt->year, NavPvt->month, NavPvt->day, NavPvt->hour, NavPvt->minute, NavPvt->seconds);

    // check if there is an actual offset before setting the RTC
    if (abs((int64_t)getNow() - (int64_t)epoch) > MAX_RTC_EPOCH_OFFSET) {
        setNow(epoch);
    }
  }

  // check that the fix is OK and that it is a 3d fix or GNSS + dead reckoning combined
  if (((NavPvt->flags & GPS_FIX_FLAGS) == GPS_FIX_FLAGS) && ((NavPvt->fixType == 3) || (NavPvt->fixType == 4))) {
    gpsData.lat = NavPvt->lat;
    gpsData.lon = NavPvt->lon;
    gpsData.numSV = NavPvt->numSV;
    isGpsDataNew = true;
  }
}

bool getGpsFix(uint32_t timeout) {
  debugPrintln("Searching for gps fix...");
  if (!isGpsInitialized) {
      debugPrintln("GPS is not initialized, exiting...");

      return false;
  }

  bool isSuccessful = false;
  setGpsActive(true);

  gpsData.numSV = 0;
  uint32_t startTime = getNow();
  while (((getNow() - startTime) <= timeout) && (gpsData.numSV < GPS_MIN_SAT_COUNT)) {
      sodaq_wdt_reset();
      uint16_t bytes = ublox.available();

      if (bytes) {
          rtcEpochDelta = 0;
          isGpsDataNew = false;
          ublox.GetPeriodic(bytes); // calls the delegate method for passing results

          startTime += rtcEpochDelta; // just in case the clock was changed (by the delegate in ublox.GetPeriodic)

          // isGpsDataNew guarantees at least a 3d fix or GNSS + dead reckoning combined
          // and is good enough to keep, but the while loop should keep trying until timeout or sat count larger than set
          if (isGpsDataNew) {
              isSuccessful = true;
          }
      }
  }

  setGpsActive(false); // turn off gps as soon as it is not needed

  if (isSuccessful) {
    debugPrintln("found gps fix");
  } else {
    debugPrintln("did not found gps fix");
  }

  return isSuccessful;
}

bool getGpsCoordinates() {
  debugPrintln("Updating gps location");

  if (getGpsFix(GPS_DEFAULT_TIMEOUT)) {
    int32_t lat = gpsData.lat;
    int32_t lon = gpsData.lon;

    myData.addLocation(lat, lon);
    // myData.commit();

    return true;
  }

  return false;
}

bool transmitLastData() {
  debugPrintln("Transmitting last location over LoRa.");

  uint8_t sendSize;
  uint8_t sendBuffer[51];

  setLedColor(CYAN);

  sendSize = myData.getLastData(&sendBuffer[0]);

  LoRa.setSpreadingFactor(sfState);
  debugPrint("\t- transmission with sf: ");
  debugPrintln(sfState);

  uint8_t result = LoRa.transmit(sendBuffer, sendSize);

  debugPrint("\t -data: ");
  for (uint8_t i = 0; i < sendSize; i++) {
    debugPrint((char)NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(sendBuffer[i])));
    debugPrint((char)NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(sendBuffer[i])));
  }
  debugPrintln();

  if (result == 0) {
    switch (sfState) {
      case SF7: {
        myData.setSF(MyData::SF7b);
        sfState = SF12;
        break;
      }
      case SF8: {
        myData.setSF(MyData::SF8b);
        sfState = SF7;
        break;
      }
      case SF9: {
        myData.setSF(MyData::SF9b);
        sfState = SF8;
        break;
      }
      case SF10: {
        myData.setSF(MyData::SF10b);
        sfState = SF9;
        break;
      }
      case SF11: {
        myData.setSF(MyData::SF11b);
        sfState = SF10;
        break;
      }
      case SF12: {
        myData.setSF(MyData::SF12b);
        sfState = SF11;
        break;
      }
    }
    myData.commit();

    return true;
  }

  return false;
}

/**
   Prints a boot-up message that includes project name, version,
   and Cpu reset cause.
*/
static void printBootUpMessage(Stream& stream) {
  stream.println("** " PROJECT_NAME " - " VERSION " **");

  getHWEUI();
  stream.print("LoRa HWEUI: ");
  for (uint8_t i = 0; i < sizeof(loraHWEui); i++) {
    stream.print((char)NIBBLE_TO_HEX_CHAR(HIGH_NIBBLE(loraHWEui[i])));
    stream.print((char)NIBBLE_TO_HEX_CHAR(LOW_NIBBLE(loraHWEui[i])));
  }
  stream.println();

  stream.print(" -> ");
  printCpuResetCause(stream);

  stream.println();
}

static void printMyDataMessage(Stream& stream) {
  stream.println("** My Saved Data **");
  stream.print("Number of saved measurements: ");
  stream.println(myData.getNumberOfMeasurements());

  if (myData.getNumberOfMeasurements() > 0) {
    unsigned long start = getNow();
    while ((getNow() - start) <= SETTINGS_DEFAULT_TIMEOUT && stream.available() == 0) {
      sodaq_wdt_reset();
    }
    String command = stream.readString();
    command.trim();
    if (command.compareTo("read data") == 0) {
      myData.print(stream);
    } else if (command.compareTo("clear data") == 0) {
      myData.reset();
      myData.setPower(DEFAULT_POWER);
      myData.commit();
    }
  }

  stream.println();
}

/**
   Prints the cause of the last reset to the given stream.

   It uses the PM->RCAUSE register to detect the cause of the last reset.
*/
static void printCpuResetCause(Stream& stream) {
  stream.print("CPU reset by");

  if (PM->RCAUSE.bit.SYST) {
    stream.print(" Software");
  }

  // Syntax error due to #define WDT in CMSIS/4.0.0-atmel/Device/ATMEL/samd21/include/samd21j18a.h
  // if (PM->RCAUSE.bit.WDT) {
  if ((PM->RCAUSE.reg & PM_RCAUSE_WDT) != 0) {
    stream.print(" Watchdog");
  }

  if (PM->RCAUSE.bit.EXT) {
    stream.print(" External");
  }

  if (PM->RCAUSE.bit.BOD33) {
    stream.print(" BOD33");
  }

  if (PM->RCAUSE.bit.BOD12) {
    stream.print(" BOD12");
  }

  if (PM->RCAUSE.bit.POR) {
    stream.print(" Power On Reset");
  }

  stream.print(" [");
  stream.print(PM->RCAUSE.reg);
  stream.println("]");
}

void getHWEUI() {
  // only read the HWEUI once
  if (!isLoraHWEuiInitialized) {
    if (initLora(LORA_INIT_SKIP_CONSOLE_MESSAGES, LORA_INIT_SKIP_JOIN)) {
      sodaq_wdt_safe_delay(10);
      uint8_t len = LoRa.getHWEUI(loraHWEui, sizeof(loraHWEui));
      if (len == sizeof(loraHWEui)) {
        isLoraHWEuiInitialized = true;
      }
    }
  }
}
