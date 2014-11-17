/**
  ******************************************************************************
  * @file    voodoospark.cpp
  * @author  Chris Williams
  * @version V2.4.0
  * @date    1-June-2014
  * @brief   Exposes the firmware level API through a TCP Connection initiated
  *          to the spark device
  ******************************************************************************
  */
#include "application.h"

// allow use of itoa() in this scope
extern char* itoa(int a, char* buffer, unsigned char radix);

#define DEBUG 0
#define PORT 48879
#define MAX_DATA_BYTES 64

// table of action codes
// to do: make this an enum?
#define PIN_MODE                    0x00
#define DIGITAL_WRITE               0x01
#define ANALOG_WRITE                0x02
#define DIGITAL_READ                0x03
#define ANALOG_READ                 0x04
#define REPORTING                   0x05
#define SET_SAMPLE_INTERVAL         0x06
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
// #define WIRE_BEGIN                  0x30
// #define WIRE_REQUEST_FROM           0x31
// #define WIRE_BEGIN_TRANSMISSION     0x32
// #define WIRE_END_TRANSMISSION       0x33
// #define WIRE_WRITE                  0x34
// #define WIRE_AVAILABLE              0x35
// #define WIRE_READ                   0x36
/* NOTE GAP */
#define SERVO_WRITE                 0x41
#define ACTION_RANGE                0x46

uint8_t bytesToExpectByAction[] = {
  // digital/analog I/O
  2,    // PIN_MODE
  2,    // DIGITAL_WRITE
  2,    // ANALOG_WRITE
  1,    // DIGITAL_READ
  1,    // ANALOG_READ
  2,    // REPORTING
  1,    // SET_SAMPLE_INTERVAL
  // gap from 0x07-0x0f
  0,    // 0x07
  0,    // 0x08
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
  1,    // WIRE_BEGIN
  3,    // WIRE_REQUEST_FROM
  1,    // WIRE_BEGIN_TRANSMISSION
  1,    // WIRE_END_TRANSMISSION
  1,    // WIRE_WRITE  -- variable length message!
  0,    // WIRE_AVAILABLE
  0,    // WIRE_READ
  // gap from 0x37-0x3f
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

byte buffer[MAX_DATA_BYTES];
byte cached[4];
byte reporting[20];

int reporters = 0;
int bytesRead = 0;
int bytesExpecting = 0;
int action, available;

unsigned long lastms;
unsigned long nowms;
unsigned long sampleInterval = 100;
unsigned long SerialSpeed[] = {
  600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
};

/*
  PWM/Servo support is CONFIRMED available on:

  D0, D1, A0, A1, A4, A5, A6, A7

  Allocate 8 servo objects:
 */
Servo servos[8];
/*
  The Spark board can only support PWM/Servo on specific pins, so
  based on the pin number, determine the servo index for the allocated
  servo object.
 */
int ToServoIndex(int pin) {
  // D0, D1
  if (pin == 0 || pin == 1) return pin;
  // A0, A1
  if (pin == 10 || pin == 11) return pin - 8;
  // A4, A5, A6, A7
  if (pin >= 14) return pin - 10;
}

void send(int action, int pin, int value) {
  // See https://github.com/voodootikigod/voodoospark/issues/20
  // to understand why the send function splits values
  // into two 7-bit bytes before sending.
  //
  int lsb = value & 0x7f;
  int msb = value >> 0x07 & 0x7f;

  server.write(action);
  server.write(pin);

  // Send the LSB
  server.write(lsb);
  // Send the MSB
  server.write(msb);

  // #ifdef DEBUG
  // Serial.print("SENT: ");
  // Serial.print(value);
  // Serial.print(" -> [ ");
  // Serial.print(lsb);
  // Serial.print(", ");
  // Serial.print(msb);
  // Serial.println(" ]");
  // #endif
}

void report() {
  if (isConnected) {
    for (int i = 0; i < 20; i++) {
      if (reporting[i]) {
        #ifdef DEBUG
        Serial.print("Reporting: ");
        Serial.print(i, DEC);
        Serial.println(reporting[i], DEC);
        #endif

        int dr = (reporting[i] & 1);
        int ar = (reporting[i] & 2);

        if (i < 10 && dr) {
          send(DIGITAL_READ, i, digitalRead(i));
        } else {
          if (dr) {
            send(DIGITAL_READ, i, digitalRead(i));
          } else {
            if (ar) {
              int adc = analogRead(i);
              #ifdef DEBUG
              Serial.print("Analog Report (pin, adc): ");
              Serial.print(i, DEC);
              Serial.print(" ");
              Serial.println(adc, DEC);
              #endif
              send(ANALOG_READ, i, adc);
            }
          }
        }
      }
    }
  }
}

void restore() {
  #ifdef DEBUG
  Serial.println("--------------RESTORING");
  #endif

  hasAction = false;
  isConnected = false;

  reporters = 0;
  bytesRead = 0;
  bytesExpecting = 0;

  lastms = 0;
  nowms = 0;
  sampleInterval = 100;

  memset(&buffer[0], 0, MAX_DATA_BYTES);
  memset(&cached[0], 0, 4);
  memset(&reporting[0], 0, 20);

  for (int i = 0; i < 8; i++) {
    if (servos[i].attached()) {
      servos[i].detach();
    }
  }
}

void setup() {

  server.begin();
  netapp_ipconfig(&ip_config);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  IPAddress ip = WiFi.localIP();
  static char ipAddress[24] = "";
  char octet[5];

  itoa(ip[0], octet, 10); strcat(ipAddress, octet); strcat(ipAddress, ".");
  itoa(ip[1], octet, 10); strcat(ipAddress, octet); strcat(ipAddress, ".");
  itoa(ip[2], octet, 10); strcat(ipAddress, octet); strcat(ipAddress, ".");
  itoa(ip[3], octet, 10); strcat(ipAddress, octet); strcat(ipAddress, ":");
  itoa(PORT, octet, 10);  strcat(ipAddress, octet);

  Spark.variable("endpoint", ipAddress, STRING);
}

void processInput() {
  int pin, mode, val, type, speed, address, stop, len, k, i;
  int byteCount = bytesRead;

  #ifdef DEBUG
  Serial.println("----------processInput----------");
  Serial.print("Bytes Available: ");
  Serial.println(bytesRead, DEC);

  for (i = 0; i < bytesRead; i++) {
    Serial.print(i, DEC);
    Serial.print(": ");
    Serial.println(buffer[i], DEC);
  }
  #endif

  // Only check if buffer[0] is possibly an action
  // when there is no action in progress.
  if (hasAction == false) {
    if (buffer[0] < ACTION_RANGE) {
      hasAction = true;
      bytesExpecting = bytesToExpectByAction[action] + 1;
    }
  }

  #ifdef DEBUG
  Serial.print("Bytes Expecting: ");
  Serial.println(bytesExpecting, DEC);
  Serial.print("Bytes Read: ");
  Serial.println(bytesRead, DEC);
  #endif

  // When the first byte of buffer is an action and
  // enough bytes are read, begin processing the action.
  if (hasAction && bytesRead >= bytesExpecting) {

    action = buffer[0];

    #ifdef DEBUG
    Serial.print("Action received: ");
    Serial.println(action, DEC);
    #endif


    // Copy the expected bytes into the cache and shift
    // the unused bytes to the beginning of the buffer
    for (k = 0; k < byteCount; k++) {
      // Cache the bytes that we're expecting for
      // this action.
      if (k < bytesExpecting) {
        cached[k] = buffer[k];

        // Reduce the bytesRead by the number of bytes "taken"
        bytesRead--;

        #ifdef DEBUG
        Serial.print("Cached: ");
        Serial.print(k, DEC);
        Serial.println(cached[k], DEC);
        #endif
      }

      // Shift the unused buffer to the front
      buffer[k] = buffer[k + bytesExpecting];
    }

    // Proceed with action processing
    switch (action) {
      case PIN_MODE:  // pinMode
        pin = cached[1];
        mode = cached[2];
        #ifdef DEBUG
        Serial.print("PIN received: ");
        Serial.println(pin);
        Serial.print("MODE received: ");
        Serial.println(mode, HEX);
        #endif

        if (servos[ToServoIndex(pin)].attached()) {
          servos[ToServoIndex(pin)].detach();
        }

        if (mode == 0x00) {
          pinMode(pin, INPUT);
        } else if (mode == 0x02) {
          pinMode(pin, INPUT_PULLUP);
        } else if (mode == 0x03) {
          pinMode(pin, INPUT_PULLDOWN);
        } else if (mode == 0x01) {
          pinMode(pin, OUTPUT);
        } else if (mode == 0x04) {
          pinMode(pin, OUTPUT);
          servos[ToServoIndex(pin)].attach(pin);
        }
        break;

      case DIGITAL_WRITE:  // digitalWrite
        pin = cached[1];
        val = cached[2];
        #ifdef DEBUG
        Serial.print("PIN received: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE received: ");
        Serial.println(val, HEX);
        #endif
        digitalWrite(pin, val);
        break;

      case ANALOG_WRITE:  // analogWrite
        pin = cached[1];
        val = cached[2];
        #ifdef DEBUG
        Serial.print("PIN received: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE received: ");
        Serial.println(val, HEX);
        #endif
        analogWrite(pin, val);
        break;

      case DIGITAL_READ:  // digitalRead
        pin = cached[1];
        val = digitalRead(pin);
        #ifdef DEBUG
        Serial.print("PIN received: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE sent: ");
        Serial.println(val, HEX);
        #endif
        send(0x03, pin, val);
        break;

      case ANALOG_READ:  // analogRead
        pin = cached[1];
        val = analogRead(pin);
        #ifdef DEBUG
        Serial.print("PIN received: ");
        Serial.println(pin, DEC);
        Serial.print("VALUE sent: ");
        Serial.println(val, HEX);
        #endif
        send(0x04, pin, val);
        break;

      case REPORTING: // Add pin to
        reporters++;
        pin = cached[1];
        val = cached[2];
        #ifdef DEBUG
        Serial.print("READ: ");
        Serial.print(pin, DEC);
        Serial.println(val, HEX);
        #endif
        reporting[pin] = val;
        break;

      case SET_SAMPLE_INTERVAL: // set the sampling interval in ms
        sampleInterval = cached[1];

        // Lower than ~100ms will likely crash the spark,
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

      // // Wire API
      // case WIRE_BEGIN:  // Wire.begin
      //   address = cached[1];
      //   if (address == 0) {
      //     Wire.begin();
      //   } else {
      //     Wire.begin(address);
      //   }
      //   break;

      // case WIRE_REQUEST_FROM:  // Wire.requestFrom
      //   address = cached[1];
      //   val = cached[2];
      //   stop = cached[3];
      //   Wire.requestFrom(address, val, stop);
      //   break;

      // case WIRE_BEGIN_TRANSMISSION:  // Wire.beginTransmission
      //   address = cached[1];
      //   Wire.beginTransmission(address);
      //   break;

      // case WIRE_END_TRANSMISSION:  // Wire.endTransmission
      //   stop = cached[1];
      //   val = Wire.endTransmission(stop);
      //   server.write(0x33);    // could be (action)
      //   server.write(val);
      //   break;

      // case WIRE_WRITE:  // Wire.write
      //   len = cached[1];
      //   uint8_t wireData[len];

      //   for (i = 0; i< len; i++) {
      //     wireData[i] = cached[1];
      //   }
      //   val = Wire.write(wireData, len);

      //   server.write(0x34);    // could be (action)
      //   server.write(val);
      //   break;

      // case WIRE_AVAILABLE:  // Wire.available
      //   val = Wire.available();
      //   server.write(0x35);    // could be (action)
      //   server.write(val);
      //   break;

      // case WIRE_READ:  // Wire.read
      //   val = Wire.read();
      //   server.write(0x36);    // could be (action)
      //   server.write(val);
      //   break;

      case SERVO_WRITE:
        pin = cached[1];
        val = cached[2];
        #ifdef DEBUG
        Serial.print("PIN: ");
        Serial.println(pin);
        Serial.print("WRITING TO SERVO: ");
        Serial.println(val);
        #endif
        servos[ToServoIndex(pin)].write(val);
        break;

      default: // noop
        break;
    } // <-- This is the end of the switch

    memset(&cached[0], 0, 4);

    // Reset hasAction flag (no longer needed for this opertion)
    // action and byte read expectation flags
    hasAction = false;
    bytesExpecting = 0;

    // If there were leftover bytes available,
    // call processInput. This mechanism will continue
    // until there are no bytes available.
    if (bytesRead > 0) {
      #ifdef DEBUG
      Serial.print("# Unprocessed Bytes: ");
      Serial.println(bytesRead, DEC);
      #endif

      available = bytesRead;
      processInput();
    }
  }
}

void loop() {
  if (client.connected()) {

    if (!isConnected) {
      restore();
      #ifdef DEBUG
      Serial.println("--------------CONNECTED--------------");
      #endif
    }

    isConnected = true;

    // Process incoming bytes first
    available = client.available();

    if (available > 0) {
      // Move all available bytes into the buffer,
      // this avoids building up back pressure in
      // the client byte stream.
      for (int i = 0; i < available && i < MAX_DATA_BYTES - bytesRead; i++) {
        buffer[bytesRead++] = client.read();
      }

      #ifdef DEBUG
      Serial.println("--------------PROCESSING NEW DATA--------------");
      #endif

      processInput();
    }

    // Reporting must be limited to every ~100ms
    // Otherwise the spark becomes unreliable and
    // exhibits a higher crash frequency.
    nowms = millis();
    if (nowms - lastms > sampleInterval && reporters > 0) {
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
