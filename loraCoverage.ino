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
#include <Sodaq_RN2483_internal.h>
#include <Sodaq_RN2483.h>
#include <Sodaq_UBlox_GPS.h>
#include <Utils.h>

#define DEBUG
//#define DEBUGLORA
//#define DEBUGGPS
#define RESEARCH

#define LOWPOWER false
#define MINNUMSAT 4
#define SIZE 7
#define GPSTIMEOUT 60000UL
#define CONFIRMMODETIMEOUT 10000UL
#define CONFIRMSFTIMEOUT 10000UL
#define WALKINGTIMEOUT 30000UL

#define SerialLoRa Serial1
#define BAUDRATE 57600

// ABP
const uint8_t devAddr[4] = { 0x26, 0x01, 0x11, 0x11 };
const uint8_t appSKey[16] = { 0x9E, 0x92, 0x3F, 0x25, 0x04, 0xB2, 0x7D, 0x25, 0xDB, 0x7A, 0x49, 0xA9, 0xBE, 0x2D, 0x04, 0xBF };
const uint8_t nwkSKey[16] = { 0x25, 0x5B, 0x17, 0xF5, 0x61, 0x61, 0x73, 0x4C, 0x46, 0xEE, 0xD2, 0x3E, 0xB3, 0xFA, 0x79, 0x2D };
//uint8_t testPayload[] = { 0X03, 0X0A, 0X39, 0XD7, 0X00, 0X47, 0XEC, 0X91 };

// Led colors
enum LedColor {
    NONE = 0,
    RED,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    CYAN,
    WHITE,
};

// Modes
enum Mode {
  MANUAL = 0,
  WALKING,
};

// States
enum main_state {
  IDLE = 0,
  IDLE_W,
  GPS_SCAN,
  GPS_SCAN_W,
  LORA_TX,
  LORA_TX_W,
  ERROR,
  ERROR_W,
};

enum mode_state {
  MODE_IDLE = 0,
  MODE_IDLE_W,
  MODE_MANUAL,
  MODE_MANUAL_W,
  MODE_WALKING,
  MODE_WALKING_W,
  MODE_SET,
  MODE_SET_W,
};

enum sf_state {
  SF_IDLE = 0,
  SF_IDLE_W,
  SF_7,
  SF_7_W,
  SF_8,
  SF_8_W,
  SF_9,
  SF_9_W,
  SF_10,
  SF_10_W,
  SF_11,
  SF_11_W,
  SF_12,
  SF_12_W,
  SF_SET,
  SF_SET_W,
};

volatile bool buttonFlag;
volatile bool buttonState;

static bool isModeInitialized;
static bool isSFInitialized;
static bool isLoRaInitialized;
static bool isGPSInitialized;

main_state mState;
mode_state modeState;
Mode mode;
sf_state sfState;
bool isAdrOn = false;
uint8_t defaultLoRaPort = 0;
uint8_t spreadingFactor = 12  ;
uint8_t powerIndex = 1;

void setup();
void setupButton();
bool setupGPS();
bool setupLoRa();
void setupMode();
void setupSF();
void loop();
void buttonHandler();
bool transmit(uint8_t*, uint8_t, int16_t);
void setLoRaActive(bool );
void setLedColor(LedColor);
void blinkLedColor(LedColor, uint8_t);

void setup() {
  // show that the device is in config mode
  setLedColor(RED);

  while ((!SerialUSB) && (millis() < 10000)) {
    // Wait for SerialUSB or start after 10 seconds
  }

  // setup serial connection to usb
  SerialUSB.begin(BAUDRATE);
  
  // setup serial connection to RN2483 module
  SerialLoRa.begin(LoRaBee.getDefaultBaudRate());
  
  // setup button
  setupButton();

  // setup mode
  setupMode();

  // setup sf
  setupSF();

  // setup lora connection
  isLoRaInitialized = setupLoRa();

  if (isLoRaInitialized) {
    setLedColor(YELLOW);
  }

  if (LOWPOWER) {
    setLoRaActive(false);
  }

  // setup gps
  isGPSInitialized = setupGPS();
  
  if (isGPSInitialized) {
    setLedColor(GREEN);
  }

  SerialUSB.print("device address: ");
  for (uint8_t i = 0; i < sizeof(devAddr); i++) {
    if (devAddr[i] < 0x10) {
      SerialUSB.print("0");
    }
    SerialUSB.print(devAddr[i], HEX);
  }
  SerialUSB.println("");
  SerialUSB.print("spreading factor: SF");
  SerialUSB.println(spreadingFactor);
  SerialUSB.println("");
}

void setupButton() {
  pinMode(BUTTON, INPUT_PULLUP);
  attachInterrupt(BUTTON, buttonHandler, FALLING);
}

bool setupGPS() {
  bool result;
  sodaq_gps.init(GPS_ENABLE);

#ifdef DEBUGGPS
  sodaq_gps.setDiag(SerialUSB);
#endif

  //sodaq_gps.setMinNumSatellites(MINNUMSAT);
  result = sodaq_gps.scan(!LOWPOWER, 900000L);

  return result;
}

bool setupLoRa() {
  bool result;

#ifdef DEBUGLORA
  LoRaBee.setDiag(SerialUSB);
#endif

  result = LoRaBee.initABP(SerialLoRa, devAddr, appSKey, nwkSKey, isAdrOn);

// disbable duty cycle limits
#ifdef RESEARCH
  LoRaBee.sendCommand("mac set ch dcycle 0 7");
  LoRaBee.sendCommand("mac set ch dcycle 1 7");
  LoRaBee.sendCommand("mac set ch dcycle 2 7");
  LoRaBee.sendCommand("mac set ch dcycle 3 7");
  LoRaBee.sendCommand("mac set ch dcycle 4 7");
  LoRaBee.sendCommand("mac set ch dcycle 5 7");
  LoRaBee.sendCommand("mac set ch dcycle 6 7");
  LoRaBee.sendCommand("mac set ch dcycle 7 7");
  LoRaBee.sendCommand("mac set ch dcycle 8 7");
#endif

  if (result) {
#ifdef DEBUG
    SerialUSB.println("connection to the network was successful");
#endif
    LoRaBee.setSpreadingFactor(spreadingFactor);
    LoRaBee.setPowerIndex(powerIndex);
  } else {
#ifdef DEBUG
    SerialUSB.println("connection to the network failed");
#endif
  }

  return result;
}

void setupMode() {
  static unsigned long start;

#ifdef DEBUG
  SerialUSB.println("mode selection:");
  SerialUSB.println("[press the button to switch between modes");
  SerialUSB.println("wait 10 seconds to confirm the mode]");
#endif
  
  while (!isModeInitialized) {
    switch(modeState) {
      case MODE_IDLE: {
        modeState = MODE_IDLE_W;
        break;
      }
      case MODE_IDLE_W: {
        modeState = MODE_MANUAL;
        break;
      }
      case MODE_MANUAL: {
        mode = MANUAL;
        setLedColor(WHITE);
#ifdef DEBUG
        SerialUSB.println("mode: manual..");
#endif
        modeState = MODE_MANUAL_W;
        start = millis();
        break;
      }
      case MODE_MANUAL_W: {
        if (buttonFlag) {
          modeState = MODE_WALKING;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMMODETIMEOUT) {
          modeState = MODE_SET;
        }
        break;
      }
      case MODE_WALKING: {
        mode = WALKING;
        setLedColor(MAGENTA);
        modeState = MODE_WALKING_W;
        start = millis();
#ifdef DEBUG
        SerialUSB.println("mode: walking..");
#endif
        break;
      }
      case MODE_WALKING_W: {
        if (buttonFlag) {
          modeState = MODE_MANUAL;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMMODETIMEOUT) {
          modeState = MODE_SET;
        }
        break;
      }
      case MODE_SET: {
#ifdef DEBUG
        SerialUSB.println("mode configured");
#endif
        isModeInitialized = true;
        blinkLedColor(GREEN, 3);
        modeState = MODE_SET_W;
        break;
      }
      case MODE_SET_W: {
        break;
      }
      default: {
        setLedColor(NONE);
      }
    }
  }  
}

void setupSF() {
  static unsigned long start;
  
#ifdef DEBUG
  SerialUSB.println("sf selection:");
  SerialUSB.println("[press the button to switch between sf's");
  SerialUSB.println("wait 10 seconds to confirm the sf]");
#endif

  while (!isSFInitialized) {
    switch(sfState) {
      case SF_IDLE: {
        sfState = SF_IDLE_W;
        break;
      }
      case SF_IDLE_W: {
        sfState = SF_7;
        break;
      }
      case SF_7: {
        spreadingFactor = 7;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_7_W;
        start = millis();
        break;
      }
      case SF_7_W: {
        if (buttonFlag) {
          sfState = SF_8;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_8: {
        spreadingFactor = 8;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_8_W;
        start = millis();
        break;
      }
      case SF_8_W: {
        if (buttonFlag) {
          sfState = SF_9;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_9: {
        spreadingFactor = 9;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_9_W;
        start = millis();
        break;
      }
      case SF_9_W: {
        if (buttonFlag) {
          sfState = SF_10;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_10: {
        spreadingFactor = 10;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_10_W;
        start = millis();
        break;
      }
      case SF_10_W: {
        if (buttonFlag) {
          sfState = SF_11;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_11: {
        spreadingFactor = 11;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_11_W;
        start = millis();
        break;
      }
      case SF_11_W: {
        if (buttonFlag) {
          sfState = SF_12;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_12: {
        spreadingFactor = 12;
#ifdef DEBUG
        SerialUSB.print("sf: ");
        SerialUSB.println(spreadingFactor);
#endif
        blinkLedColor(YELLOW, spreadingFactor - 6);
        sfState = SF_12_W;
        start = millis();
        break;
      }
      case SF_12_W: {
        if (buttonFlag) {
          sfState = SF_7;
          buttonFlag = false;
        } else if (millis() >= start + CONFIRMSFTIMEOUT) {
          sfState = SF_SET;
        }
        break;
      }
      case SF_SET: {
#ifdef DEBUG
        SerialUSB.println("sf configured");
#endif
        isSFInitialized = true;
        blinkLedColor(GREEN, 3);
        sfState = SF_SET_W;
        break;
      }
      case SF_SET_W: {
        break;
      }
      default: {
        setLedColor(RED);
      }
    }
  }
}

void loop() {
  static uint8_t data[SIZE];
  static unsigned long start;

  if (!isModeInitialized || !isLoRaInitialized || !isGPSInitialized) {
    mState = ERROR;
  }

  switch(mState) {
    case IDLE: {
      memcpy(data, 0, sizeof(data));
      setLedColor(GREEN);
      mState = IDLE_W;
      start = millis();
      break;
    }
    case IDLE_W: {
      switch(mode) {
        case MANUAL: {
          if (buttonFlag) {
            mState = GPS_SCAN;
            buttonFlag = false;
          }
          break;
        }
        case WALKING: {
          if (millis() >= start + WALKINGTIMEOUT) {
            mState = GPS_SCAN;
          }
          break;
        }
        default: {
          mState = ERROR;
        }
      }
      break;
    }
    case GPS_SCAN: {
      setLedColor(CYAN);
      if (sodaq_gps.scan(!LOWPOWER, GPSTIMEOUT)) {
        int32_t lat = sodaq_gps.getLat() * 10000;
        int32_t lon = sodaq_gps.getLon() * 10000;
        data[0] = lat >> 16;
        data[1] = lat >> 8;
        data[2] = lat;
        data[3] = lon >> 16;
        data[4] = lon >> 8;
        data[5] = lon;
        data[6] = powerIndex;
        for (uint8_t i = 0; i < sizeof(data); i++) {
          if (data[i] < 0x10) {
            SerialUSB.print("0");
          }
          SerialUSB.print(data[i], HEX);
        }
        SerialUSB.println("");
        mState = GPS_SCAN_W;
      } else {
        mState = ERROR;
      }
      break;
    }
    case GPS_SCAN_W: {
      mState = LORA_TX;
      break;
    }
    case LORA_TX: {
      setLedColor(BLUE);
      if (transmit(data, sizeof(data), 1)) {
        mState = LORA_TX_W;
      } else {
        mState = ERROR;
      }
      break;
    }
    case LORA_TX_W: {
      mState = IDLE;
      break;
    }
    case ERROR: {
      blinkLedColor(RED, 10);
      mState = ERROR_W;
      break;
    }
    case ERROR_W: {
      mState = IDLE;
      break;
    }
    default: {
      setLedColor(WHITE);
    }
  }
}

void buttonHandler() {
  buttonFlag = true;
}

bool transmit(uint8_t* buffer, uint8_t size, int16_t overrideLoRaPort) {
  bool result = false;
  
  if (!isLoRaInitialized) {
    SerialUSB.println("transmitting LoRa message without initialization..");
    return false;
  }

  setLoRaActive(true);

  uint16_t port = overrideLoRaPort > -1 ? overrideLoRaPort : defaultLoRaPort;

  result = LoRaBee.send(port, buffer, size);
#ifdef DEBUG
  switch (result) {
    case NoError: {
      SerialUSB.println("successful transmission");
      break;
    }
    case NoResponse: {
      SerialUSB.println("error while transmitting: no response");
      break;
    }
    case Timeout: {
      SerialUSB.println("error while transmitting: internal error");
      break;
    }
    case InternalError: {
      SerialUSB.println("error while transmitting: no acknowledgment");
      break;
    }
    case NetworkFatalError: {
      SerialUSB.println("error while transmitting: network fatal error");
      break;
    }
    case Busy: {
      SerialUSB.println("error while transmitting: busy");
      break;
    }
    case NotConnected: {
      SerialUSB.println("error while transmitting: not connected");
      break;
    }
    case NoAcknowledgment: {
      SerialUSB.println("error while transmitting: no acknowledgment");
      break;
    }
    case PayloadSizeError: {
      SerialUSB.println("error while transmitting: payload size error");
      break;
    }
    default: {
      SerialUSB.print("unexpected error occured: ");
      SerialUSB.println(result);
      break;
    }
  }
#endif

  if (LOWPOWER) {
    setLoRaActive(false);
  }

  if (result == 0) {
    return true;
  } else {
    return false;
  }
}

void setLoRaActive(bool on) {
  if (on) {
    LoRaBee.wakeUp();
  } else {
    LoRaBee.sleep();
  }
}

void setLedColor(LedColor color) {
    pinMode(LED_RED, OUTPUT);
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);

    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_BLUE, HIGH);

    switch (color) {
    case NONE:
        break;
    case RED:
        digitalWrite(LED_RED, LOW);
        break;
    case GREEN:
        digitalWrite(LED_GREEN, LOW);
        break;
    case BLUE:
        digitalWrite(LED_BLUE, LOW);
        break;
    case YELLOW:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, LOW);
        break;
    case MAGENTA:
        digitalWrite(LED_BLUE, LOW);
        digitalWrite(LED_RED, LOW);
        break;
    case CYAN:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    case WHITE:
        digitalWrite(LED_GREEN, LOW);
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_BLUE, LOW);
        break;
    default:
        break;
    }
}

void blinkLedColor(LedColor color, uint8_t times) {
  while (times-- > 0) {
    setLedColor(NONE);
    delay(500);
    setLedColor(color);
    delay(500);
  }
  setLedColor(NONE);
}

