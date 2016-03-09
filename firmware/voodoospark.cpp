/**
  ******************************************************************************
  * @file    voodoospark.cpp
  * @author  Chris Williams
  * @version V4.0.0
  * @date    08-March-2016
  * @brief   Exposes the firmware level API through a TCP Connection initiated
  *          to the Particle devices (Core and Photon)
  ******************************************************************************
  Copyright (c) 2016 Chris Williams (voodootikigod)  All rights reserved.

  Permission is hereby granted, free of charge, to any person
  obtaining a copy of this software and associated documentation
  files (the "Software"), to deal in the Software without
  restriction, including without limitation the rights to use,
  copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the
  Software is furnished to do so, subject to the following
  conditions:

  The above copyright notice and this permission notice shall be
  included in all copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
  OTHER DEALINGS IN THE SOFTWARE.
  ******************************************************************************
  */

#include "application.h"

#define DEBUG 0
#define PORT 48879
#define MAX_DATA_BYTES 128
#define MAX_I2C_READ_CONTINUOUS_DEVICES 8

// table of action codes
// to do: make this an enum?
#define PIN_MODE                    0x00
#define DIGITAL_WRITE               0x01
#define ANALOG_WRITE                0x02
#define DIGITAL_READ                0x03
#define ANALOG_READ                 0x04
#define REPORTING                   0x05
#define SET_SAMPLE_INTERVAL         0x06
#define INTERNAL_RGB                0x07
#define PING_READ                   0x08

/* NOTE GAP */
// #define SERIAL_BEGIN                0x10
// #define SERIAL_END                  0x11
// #define SERIAL_PEEK                 0x12
// #define SERIAL_AVAILABLE            0x13
// #define SERIAL_WRITE                0x14
// #define SERIAL_READ                 0x15
// #define SERIAL_FLUSH                0x16
/* NOTE GAP */
// #define SPI_BEGIN                   0x20
// #define SPI_END                     0x21
// #define SPI_SET_BIT_ORDER           0x22
// #define SPI_SET_CLOCK               0x23
// #define SPI_SET_DATA_MODE           0x24
// #define SPI_TRANSFER                0x25
// /* NOTE GAP */
#define I2C_CONFIG                  0x30
#define I2C_WRITE                   0x31
#define I2C_READ                    0x32
#define I2C_READ_CONTINUOUS         0x33
#define I2C_REGISTER_NOT_SPECIFIED  0xFF
/* NOTE GAP */
#define SERVO_WRITE                 0x41
#define ACTION_RANGE                0x46

#define IS_PHOTON() PLATFORM_ID == PLATFORM_PHOTON_PRODUCTION || \
                    PLATFORM_ID == PLATFORM_P1

#define IS_CORE() PLATFORM_ID == PLATFORM_SPARK_CORE || \
                    PLATFORM_ID == PLATFORM_SPARK_CORE_HD

uint8_t bytesToExpectByAction[] = {
  // digital/analog I/O
  2,    // PIN_MODE
  2,    // DIGITAL_WRITE
  2,    // ANALOG_WRITE
  1,    // DIGITAL_READ
  1,    // ANALOG_READ
  2,    // REPORTING
  2,    // SET_SAMPLE_INTERVAL
  3,    // INTERNAL_RGB
  1,    // PING_READ
  // gap from 0x09-0x0f
  0,    // 0x09
  0,    // 0x0a
  0,    // 0x0b
  0,    // 0x0c
  0,    // 0x0d
  0,    // 0x0e
  0,    // 0x0f
  // serial I/O
  2,    // SERIAL_BEGIN
  1,    // SERIAL_END
  1,    // SERIAL_PEEK
  1,    // SERIAL_AVAILABLE
  2,    // SERIAL_WRITE  -- variable length message!
  1,    // SERIAL_READ
  1,    // SERIAL_FLUSH
  // gap from 0x17-0x1f
  0,    // 0x17
  0,    // 0x18
  0,    // 0x19
  0,    // 0x1a
  0,    // 0x1b
  0,    // 0x1c
  0,    // 0x1d
  0,    // 0x1e
  0,    // 0x1f
  // SPI I/O
  0,    // SPI_BEGIN
  0,    // SPI_END
  1,    // SPI_SET_BIT_ORDER
  1,    // SPI_SET_CLOCK
  1,    // SPI_SET_DATA_MODE
  1,    // SPI_TRANSFER
  // gap from 0x26-0x2f
  0,    // 0x26
  0,    // 0x27
  0,    // 0x28
  0,    // 0x29
  0,    // 0x2a
  0,    // 0x2b
  0,    // 0x2c
  0,    // 0x2d
  0,    // 0x2e
  0,    // 0x2f
  // wire I/O
  2,    // I2C_CONFIG
  3,    // I2C_WRITE -- variable length message!
  5,    // I2C_READ
  5,    // I2C_READ_CONTINUOUS
  // gap from 0x34-0x3f
  0,    // 0x34
  0,    // 0x35
  0,    // 0x36
  0,    // 0x37
  0,    // 0x38
  0,    // 0x39
  0,    // 0x3a
  0,    // 0x3b
  0,    // 0x3c
  0,    // 0x3d
  0,    // 0x3e
  0,    // 0x3f
  0,    // 0x40
  // servo
  2,    // SERVO_WRITE
  1,    // SERVO_DETACH
};


TCPServer server = TCPServer(PORT);
TCPClient client;

bool hasAction = false;
bool isConnected = false;

byte analogReporting[20];
byte buffer[MAX_DATA_BYTES];
byte cached[64];
byte i2cRxData[64];
byte pinModeFor[20];
byte portValues[2];
byte reporting[20];

int action, available;
int bytesRead = 0;
int bytesExpecting = 0;
int reporters = 0;

signed int i2cIndex = -1;

// Default delay time between i2c read request and Wire.requestFrom()
unsigned int i2cReadDelayTime = 0;

unsigned long lastms;
unsigned long nowms;
unsigned long SerialSpeed[] = {
  600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
};
#if IS_PHOTON()
  unsigned long sampleInterval = 20;
#else
  unsigned long sampleInterval = 100;
#endif

/* i2c data */
struct I2CDevice {
  byte address;
  int reg;
  byte bytes;
};

/* Track I2C continuous read devices */
I2CDevice i2cDevices[MAX_I2C_READ_CONTINUOUS_DEVICES];

/*
  PWM/Servo support is CONFIRMED available on:

  D0, D1, A0, A1, A4, A5, A6, A7

  Allocate 8 servo objects:
 */
Servo servos[8];
/*
  The Particle devices can only support PWM/Servo on specific pins, so
  based on the pin number, determine the servo index for the allocated
  servo object.
 */
int ToServoIndex(int pin) {
  #if IS_PHOTON()
    // Pin:   D0, D1, D2, D3
    // Index:  0,  1,  2,  3
    if (pin == 0 || pin == 1 || pin == 2 || pin == 3) return pin;
    // Pin:   A4, A5, A6, A7
    // Index:  4,  5,  6,  7
    if (pin >= 14) return pin - 10;
  #else
    // Pin:   D0, D1
    // Index:  0,  1
    if (pin == 0 || pin == 1) return pin;
    // Pin:   A0, A1
    // Index:  2,  3
    if (pin == 10 || pin == 11) return pin - 8;
    // Pin:   A4, A5, A6, A7
    // Index:  4,  5,  6,  7
    if (pin >= 14) return pin - 10;
  #endif
}

void ToUInt7Array(long value, uint8_t b[]) {
  // LSB
  b[0] = value & 0x7F;
  // MSB
  b[1] = value >> 0x07 & 0x7F;
}

void send(int action, int pinOrPort, int pinOrPortValue) {
  uint8_t buf[4];
  uint8_t u7[2];

  // See https://github.com/voodootikigod/voodoospark/issues/20
  // to understand why the send function splits values
  // into two 7-bit bytes before sending.
  ToUInt7Array(pinOrPortValue, u7);

  buf[0] = action;
  buf[1] = pinOrPort;
  // LSB
  buf[2] = u7[0];
  // MSB
  buf[3] = u7[1];

  server.write(buf, 4);
}

void setup() {

  server.begin();

  #if DEBUG
  Serial.begin(115200);
  #endif

  IPAddress ip = WiFi.localIP();
  static char ipAddress[24] = "";

  // https://community.particle.io/t/network-localip-to-string-to-get-it-via-spark-variable/2581/5
  sprintf(ipAddress, "%d.%d.%d.%d:%d", ip[0], ip[1], ip[2], ip[3], PORT);

  Particle.variable("endpoint", ipAddress, STRING);
}

void readAndReportI2cData(byte address, int theRegister, byte numBytes) {
  byte data;

  #if DEBUG
  Serial.println("-------------- I2C Read and Report Data");
  Serial.print("address: 0x");
  Serial.println(address, HEX);
  Serial.print("register: 0x");
  Serial.println(theRegister, HEX);
  Serial.print("numBytes: ");
  Serial.println(numBytes, DEC);
  #endif

  // allow I2C requests that don't require a register read
  // for example, some devices using an interrupt pin to signify new data available
  // do not always require the register read so upon interrupt you call Wire.requestFrom()
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    Wire.write((byte)theRegister);
    Wire.endTransmission();
    // do not set a value of 0
    if (i2cReadDelayTime > 0) {
      // delay is necessary for some devices such as WiiNunchuck
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0xFF;  // fill the register with an impossible value
  }

  Wire.requestFrom(address, numBytes);  // all bytes are returned in requestFrom

  #if DEBUG
  // check to be sure correct number of bytes were returned by device
  if (numBytes < Wire.available()) {
    Serial.println("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Serial.println("I2C: Too few bytes received");
  }
  #endif

  i2cRxData[0] = 0x77;
  i2cRxData[1] = numBytes;
  i2cRxData[2] = address;
  i2cRxData[3] = theRegister & 0x7F;
  i2cRxData[4] = theRegister >> 0x07 & 0x7F;

  #if DEBUG
  Serial.print("Number of Bytes: ");
  Serial.println(numBytes, DEC);
  Serial.print("Address: ");
  Serial.println(address, HEX);
  Serial.print("Register: ");
  Serial.println(theRegister, HEX);
  #endif

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    data = Wire.read();

    #if DEBUG
    Serial.print("Data[");
    Serial.print(i, DEC);
    Serial.print("]: 0x");
    Serial.println(data, HEX);
    #endif

    i2cRxData[5 + i] = data;
  }

  // send address, register and received bytes
  server.write(i2cRxData, numBytes + 5);
}

/*
 * Original pulseIn Function for the Spark Core - Version 0.1.1 (Beta)
 * Copyright (2014) Timothy Brown - See: pulsein.license
 *
 * Due to the current timeout issues with Spark Cloud
 * this will return after 10 seconds, even if the
 * input pulse hasn't finished.
 *
 * Input: Trigger Pin, Trigger State
 * Output: Pulse Length in Microseconds (10uS to 10S)
 *
 * Copyright (2015) Rick Waldron (Modifications for improved accuracy on with a Photon)
 * See LICENSE-MIT
 *
 */

unsigned long pingPulseIn(uint16_t pin, uint8_t state) {
  #if (PLATFORM_ID == 0)  // Core
    STM32_Pin_Info* PIN_MAP = HAL_Pin_Map(); // Pointer required for highest access speed
  #endif

  // Cache the target's peripheral mask to speed up the loops.
  GPIO_TypeDef* portMask = (PIN_MAP[pin].gpio_peripheral);
  // Cache the target's GPIO pin mask to speed up the loops.
  uint16_t pinMask = (PIN_MAP[pin].gpio_pin);
  unsigned long numloops = 0;
  // This has to be extremely short, because we dont want to block the process
  // for longer than is really appropriate
  unsigned long maxloops = 500000;

  // Wait for the pin to enter target state while keeping track of the timeout.
  while (GPIO_ReadInputDataBit(portMask, pinMask) != state) {
    if (numloops++ == maxloops) {
      Serial.println("Timed out, never equaled state");
      return 0;
    }
  }

  unsigned long us = micros();
  unsigned long width;

  while (GPIO_ReadInputDataBit(portMask, pinMask) == state) {
    if (numloops++ == maxloops) {
      return 0;
    }

    width = micros() - us;
  }

  return width;
}


unsigned long pingRead(int pin) {
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
  delayMicroseconds(2);
  digitalWrite(pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin, LOW);

  pinMode(pin, INPUT);

  unsigned long duration = pingPulseIn((uint16_t)pin, (uint8_t)HIGH);

  return duration;
}

void cacheBuffer(int byteCount, int cacheLength) {
  // Copy the expected bytes into the cache and shift
  // the unused bytes to the beginning of the buffer
  #if DEBUG
  Serial.print("Cached: ");
  #endif

  for (int k = 0; k < byteCount; k++) {
    // Cache the bytes that we're expecting for
    // this action.
    if (k < cacheLength) {
      cached[k] = buffer[k];

      #if DEBUG
      Serial.print("0x");
      Serial.print(cached[k], HEX);
      Serial.print(", ");
      #endif
    }

    // Shift the unused buffer to the front
    buffer[k] = buffer[k + cacheLength];
  }

  #if DEBUG
  Serial.println("");
  #endif
}


void report() {
  if (isConnected) {

    #if DEBUG
    Serial.println("--------------REPORTING");
    #endif

    int pin;
    int pinValue;
    int i;

    for (int k = 0; k < 2; k++) {
      bool shouldSend = false;
      // D0-D7
      // portValues[0] = 0;
      // A0-A7
      // portValues[1] = 0;
      portValues[k] = 0;

      for (i = 0; i < 8; i++) {
        pin = (k * 10) + i;

        if (reporting[pin] == 1) {
          shouldSend = true;
          pinValue = digitalRead(pin);

          if (pinValue) {
            portValues[k] |= 1 << i;
          }
        }
      }

      if (shouldSend) {
        #if DEBUG
        Serial.print("Reporting: ");
        Serial.print(k, DEC);
        Serial.println(portValues[k], DEC);
        #endif

        send(REPORTING, k, portValues[k]);
      }
    }

    for (i = 10; i < 18; i++) {
      if (analogReporting[i] == 1) {
        int adc = analogRead(i);

        #if DEBUG
        Serial.print("Analog Reporting: ");
        Serial.print(i, DEC);
        Serial.print(": ");
        Serial.println(adc, DEC);
        #endif

        send(ANALOG_READ, i, adc);
        delay(1);
      }
    }

    // Report I2C_READ_CONTINUOUS devices
    if (i2cIndex != -1) {
      for (i = 0; i <= i2cIndex; i++) {
        readAndReportI2cData(i2cDevices[i].address, i2cDevices[i].reg, i2cDevices[i].bytes);
      }
    }
  }
}

void restore() {
  #if DEBUG
  Serial.println("--------------RESTORING");
  #endif

  hasAction = false;
  isConnected = false;

  reporters = 0;
  bytesRead = 0;
  bytesExpecting = 0;

  i2cIndex = -1;
  lastms = 0;
  nowms = 0;
  sampleInterval = 100;

  memset(&analogReporting[0], 0, 20);
  memset(&buffer[0], 0, MAX_DATA_BYTES);
  memset(&cached[0], 0, 64);
  memset(&i2cRxData[0], 0, 64);
  memset(&pinModeFor[0], 0, 20);
  memset(&portValues[0], 0, 2);
  memset(&reporting[0], 0, 20);

  for (int i = 0; i < 8; i++) {
    if (servos[i].attached()) {
      servos[i].detach();
    }
  }

  // Restore defaults.
  for (int i = 0; i < 8; i++) {
    pinMode(i, OUTPUT);
    pinMode(i + 10, INPUT);

    pinModeFor[i] = 1;
    pinModeFor[i + 10] = 0;
  }
}


void processInput() {
  int pin, mode, val, address, reg, delayTime, dataLength;
  int byteCount = bytesRead;

  unsigned long us;

  #if DEBUG
  Serial.println("--------------PROCESSING");
  #endif

  // Only check if buffer[0] is possibly an action
  // when there is no action in progress.
  if (hasAction == false) {
    if (buffer[0] < ACTION_RANGE) {
      action = buffer[0];
      bytesExpecting = bytesToExpectByAction[action] + 1;
      hasAction = true;

      #if DEBUG
      Serial.print("Bytes Read: ");
      Serial.println(bytesRead, DEC);
      Serial.print("Bytes Required: ");
      Serial.println(bytesExpecting, DEC);
      Serial.print("Bytes Remaining: ");
      Serial.println(bytesRead - bytesExpecting, DEC);
      #endif
    }
  }

  if ((bytesRead - bytesExpecting) < 0) {
    hasAction = false;
    bytesExpecting = 0;

    #if DEBUG
    Serial.println("Not Enough Bytes.");
    #endif
    return;
  }

  // When the first byte of buffer is an action and
  // enough bytes are read, begin processing the action.
  if (hasAction && bytesRead >= bytesExpecting) {

    cacheBuffer(byteCount, bytesExpecting);
    byteCount -= bytesExpecting;

    #if DEBUG
    Serial.print("ACTION: 0x");
    Serial.println(action, HEX);
    #endif

    // Proceed with action processing
    switch (action) {
      case PIN_MODE:  // pinMode
        pin = cached[1];
        mode = cached[2];
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin);
        Serial.print("MODE: ");
        Serial.println(mode, HEX);
        #endif


        if (pinModeFor[pin] != mode) {

          if (pinModeFor[pin] == 0x04) {
            servos[ToServoIndex(pin)].detach();
          }

          pinModeFor[pin] = mode;

          // The following modes were derived
          // from uses in core-firmware.
          if (mode == 0x00) {
            // INPUT
            pinMode(pin, INPUT_PULLDOWN);
          }
          if (mode == 0x01) {
            // OUTPUT
            pinMode(pin, OUTPUT);
          }
          if (mode == 0x02) {
            // ANALOG INPUT
            #if IS_CORE()
              pinMode(pin, INPUT);
            #endif
          }
          if (mode == 0x03) {
            // ANALOG (PWM) OUTPUT
            pinMode(pin, OUTPUT);
          }
          if (mode == 0x04) {
            // SERVO
            pinMode(pin, OUTPUT);
            servos[ToServoIndex(pin)].attach(pin);
          }
        }
        break;

      case DIGITAL_WRITE:  // digitalWrite
        pin = cached[1];
        val = cached[2];
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE: ");
        Serial.println(val, HEX);
        #endif
        digitalWrite(pin, val);
        break;

      case ANALOG_WRITE:  // analogWrite
        pin = cached[1];
        val = cached[2];
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE: ");
        Serial.println(val, HEX);
        #endif
        analogWrite(pin, val);
        break;

      case DIGITAL_READ:  // digitalRead
        pin = cached[1];
        val = digitalRead(pin);
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE: ");
        Serial.println(val, HEX);
        #endif
        send(0x03, pin, val);
        break;

      case ANALOG_READ:  // analogRead
        pin = cached[1];
        val = analogRead(pin);
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE: ");
        Serial.println(val, HEX);
        #endif
        send(0x04, pin, val);
        break;

      case PING_READ:
        pin = cached[1];
        us = pingRead(pin);

        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE: ");
        Serial.println(us, DEC);
        #endif

        byte duration[6];

        duration[0] = PING_READ;
        duration[1] = pin;
        duration[2] = (us >> 24) & 0xFF;
        duration[3] = (us >> 16) & 0xFF;
        duration[4] = (us >> 8) & 0xFF;
        duration[5] = us & 0xFF;

        #if DEBUG
        for (int di = 0; di < 6; di++) {
          Serial.print(duration[di]);Serial.print(" ");
        }
        #endif

        server.write(duration, 6);
        break;

      case REPORTING: // Add pin to
        pin = cached[1];
        val = cached[2];

        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin, DEC);
        Serial.print("TYPE: ");
        Serial.println(val, DEC);
        #endif

        if (analogReporting[pin] == 0 || reporting[pin] == 0) {
          reporters++;
        }

        if (val == 2) {
          analogReporting[pin] = 1;
        } else {
          reporting[pin] = 1;
        }
        break;

      case SET_SAMPLE_INTERVAL: // set the sampling interval in ms
        sampleInterval = cached[1] + (cached[2] << 7);

        #if DEBUG
        Serial.print("SET_SAMPLE_INTERVAL (2 bytes): ");
        Serial.println(sampleInterval, DEC);
        #endif

        // Lower than ~100ms will likely crash the device,
        // but
        if (sampleInterval < 20) {
          sampleInterval = 20;
        }
        break;

      // // Serial API
      // case SERIAL_BEGIN:  // serial.begin
      //   type = cached[1];
      //   speed = cached[2];
      //   if (type == 0) {
      //     Serial.begin(SerialSpeed[speed]);
      //   } else {
      //     Serial1.begin(SerialSpeed[speed]);
      //   }
      //   break;

      // case SERIAL_END:  // serial.end
      //   type = cached[1];
      //   if (type == 0) {
      //     Serial.end();
      //   } else {
      //     Serial1.end();
      //   }
      //   break;

      // case SERIAL_PEEK:  // serial.peek
      //   type = cached[1];
      //   if (type == 0) {
      //     val = Serial.peek();
      //   } else {
      //     val = Serial1.peek();
      //   }
      //   send(0x07, type, val);
      //   break;

      // case SERIAL_AVAILABLE:  // serial.available()
      //   type = cached[1];
      //   if (type == 0) {
      //     val = Serial.available();
      //   } else {
      //     val = Serial1.available();
      //   }
      //   send(0x07, type, val);
      //   break;

      // case SERIAL_WRITE:  // serial.write
      //   type = cached[1];
      //   len = cached[2];

      //   for (i = 0; i < len; i++) {
      //     if (type == 0) {
      //       Serial.write(client.read());
      //     } else {
      //       Serial1.write(client.read());
      //     }
      //   }
      //   break;

      // case SERIAL_READ: // serial.read
      //   type = cached[1];
      //   if (type == 0) {
      //     val = Serial.read();
      //   } else {
      //     val = Serial1.read();
      //   }
      //   send(0x16, type, val);
      //   break;

      // case SERIAL_FLUSH: // serial.flush
      //   type = cached[1];
      //   if (type == 0) {
      //     Serial.flush();
      //   } else {
      //     Serial1.flush();
      //   }
      //   break;

      // SPI API
      // case SPI_BEGIN:  // SPI.begin
      //   SPI.begin();
      //   break;

      // case SPI_END:  // SPI.end
      //   SPI.end();
      //   break;

      // case SPI_SET_BIT_ORDER:  // SPI.setBitOrder
      //   type = cached[1];
      //   SPI.setBitOrder((type ? MSBFIRST : LSBFIRST));
      //   break;

      // case SPI_SET_CLOCK:  // SPI.setClockDivider
      //   val = cached[1];
      //   if (val == 0) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV2);
      //   } else if (val == 1) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV4);
      //   } else if (val == 2) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV8);
      //   } else if (val == 3) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV16);
      //   } else if (val == 4) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV32);
      //   } else if (val == 5) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV64);
      //   } else if (val == 6) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV128);
      //   } else if (val == 7) {
      //     SPI.setClockDivider(SPI_CLOCK_DIV256);
      //   }
      //   break;

      // case SPI_SET_DATA_MODE:  // SPI.setDataMode
      //   val = cached[1];
      //   if (val == 0) {
      //     SPI.setDataMode(SPI_MODE0);
      //   } else if (val == 1) {
      //     SPI.setDataMode(SPI_MODE1);
      //   } else if (val == 2) {
      //     SPI.setDataMode(SPI_MODE2);
      //   } else if (val == 3) {
      //     SPI.setDataMode(SPI_MODE3);
      //   }
      //   break;

      // case SPI_TRANSFER:  // SPI.transfer
      //   val = cached[1];
      //   val = SPI.transfer(val);
      //   server.write(0x24);
      //   server.write(val);
      //   break;

      // Wire API
      case I2C_CONFIG:
        delayTime = cached[1] + (cached[2] << 7);

        #if DEBUG
        Serial.print("delayTime: ");
        Serial.println(delayTime, DEC);
        #endif

        if (delayTime > 0) {
          i2cReadDelayTime = delayTime;
        }

        pinModeFor[0] = 0x06;
        pinModeFor[1] = 0x06;

        if (!Wire.isEnabled()) {
          #if DEBUG
          Serial.println("******* Enable I2C ******");
          #endif

          Wire.begin();
        }

        break;

      case I2C_WRITE:
        address = cached[1];
        dataLength = cached[2] + (cached[3] << 7);

        #if DEBUG
        Serial.print("address: ");
        Serial.println(address, HEX);
        Serial.print("data length: ");
        Serial.println(dataLength, DEC);
        #endif

        cacheBuffer(byteCount, dataLength);
        byteCount -= dataLength;

        Wire.beginTransmission(address);
        for (byte i = 0; i < dataLength; i += 2) {
          val = cached[i] + (cached[i + 1] << 7);

          #if DEBUG
          Serial.print("data[");
          Serial.print(i, DEC);
          Serial.print("]: 0x");
          Serial.print(val, HEX);
          Serial.print(", ");
          #endif

          Wire.write(val);
        }

        #if DEBUG
        Serial.println("");
        #endif

        Wire.endTransmission();
        delayMicroseconds(70);
        break;

      case I2C_READ:
      case I2C_READ_CONTINUOUS:
        address = cached[1];
        reg = cached[2] + (cached[3] << 7);  // register
        val = cached[4] + (cached[5] << 7);  // bytes to read

        #if DEBUG
        Serial.print("address: ");
        Serial.println(address, HEX);
        Serial.print("register: ");
        Serial.println(reg, HEX);
        Serial.print("data: ");
        Serial.println(val);
        #endif

        // reg was a dummy
        if (reg == 0xFF) {
          reg = (int)I2C_REGISTER_NOT_SPECIFIED;
          #if DEBUG
          Serial.println("I2C_REGISTER_NOT_SPECIFIED");
          #endif
        }

        if (action == I2C_READ_CONTINUOUS) {
          if ((i2cIndex + 1) > MAX_I2C_READ_CONTINUOUS_DEVICES) {
            // TODO: need to add an error protocol.
            // For now, it's unlikely that this will be an issue.
          }
          i2cIndex++;
          i2cDevices[i2cIndex].address = address;
          i2cDevices[i2cIndex].reg = reg;
          i2cDevices[i2cIndex].bytes = val;
        } else {
          readAndReportI2cData(address, reg, val);
        }

        break;

      case SERVO_WRITE:
        pin = cached[1];
        val = cached[2];
        #if DEBUG
        Serial.print("PIN: ");
        Serial.println(pin);
        Serial.print("WRITING TO SERVO: ");
        Serial.println(val);
        #endif
        servos[ToServoIndex(pin)].write(val);
        break;

      case INTERNAL_RGB:
        byte red;
        byte green;
        byte blue;
        red = cached[1];
        green = cached[2];
        blue = cached[3];
        #if DEBUG
        Serial.println("WRITING TO INTERNAL RGB LED.");
        Serial.print("Red: ");
        Serial.println(red);
        Serial.print("Green: ");
        Serial.println(green);
        Serial.print("Blue: ");
        Serial.println(blue);
        #endif
        RGB.control(true);
        RGB.color(red, green, blue);
        break;

      default: // noop
        break;
    } // <-- This is the end of the switch

    memset(&cached[0], 0, 64);


    #if DEBUG
    Serial.print("Unprocessed Bytes: ");
    Serial.println(byteCount, DEC);
    #endif


    // Reset hasAction flag (no longer needed for this opertion)
    // action and byte read expectation flags
    hasAction = false;
    bytesExpecting = 0;

    // If there were leftover bytes available,
    // call processInput. This mechanism will
    // continue until the buffer is exhausted.

    bytesRead = byteCount;

    if (byteCount > 2) {
      #if DEBUG
      Serial.println("Calling processInput ");
      #endif

      processInput();
    } else {
      #if DEBUG
      Serial.println("RETURN TO LOOP!");
      #endif
    }
  }
}

void loop() {
  if (client.connected()) {

    if (!isConnected) {
      restore();
      #if DEBUG
      Serial.println("--------------CONNECTED");
      #endif
    }

    isConnected = true;

    // Process incoming bytes first
    available = client.available();

    if (available > 0) {

      int received = 0;

      #if DEBUG
      Serial.println("--------------BUFFERING AVAILABLE BYTES");
      Serial.print("Byte Offset: ");
      Serial.println(bytesRead, DEC);
      #endif

      // Move all available bytes into the buffer,
      // this avoids building up back pressure in
      // the client byte stream.
      for (int i = 0; i < available && i < MAX_DATA_BYTES - bytesRead; i++) {
        buffer[bytesRead++] = client.read();
        received++;
      }

      #if DEBUG
      Serial.print("Bytes Received: ");
      Serial.println(received, DEC);

      Serial.print("Bytes In Buffer: ");
      Serial.println(bytesRead, DEC);

      for (int i = 0; i < bytesRead; i++) {
        Serial.print(i, DEC);
        Serial.print(":0x");
        Serial.print(buffer[i], HEX);
        Serial.print(", ");
      }
      Serial.println("");
      #endif

      processInput();
    }


    // Reporting must be limited to every ~100ms
    // Otherwise the device becomes unreliable and
    // exhibits a higher crash frequency.
    nowms = millis();
    if (nowms - lastms > sampleInterval && (reporters > 0 || i2cIndex > -1)) {
      // possible just assign the value of nowms?
      lastms += sampleInterval;
      report();
    }
  } else {
    // Upon disconnection, restore init state.
    if (isConnected) {
      restore();
    }

    // If no client is yet connected, check for a new connection
    client = server.available();
  }
}
