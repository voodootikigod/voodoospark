/**
  ******************************************************************************
  * @file    voodoospark.cpp
  * @author  Chris Williams
  * @version V2.0.0
  * @date    07-May-2014
  * @brief   Exposes the firmware level API through a TCP Connection intiated
  *          to the spark device
  ******************************************************************************
  Copyright (c) 2014 Chris Williams  All rights reserved.

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

static const bool DEBUG = 1;

static const int PORT = 48879; // 0xbeef

// TCPClient client;
TCPServer server = TCPServer(PORT);
TCPClient client;

byte reading[20];
byte previous[20];
Servo servos[20];

long SerialSpeed[] = { 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200 };


void reset() {
  for (int i = 0; i < 20; i++) {
    reading[i] = 0;
    previous[i] = 0;
  }
}




void send(int action, int pin, int value) {
  if (previous[pin] != value) {
    server.write(action);
    server.write(pin);
    server.write(value);
  }
  previous[pin] = value;
}

void report() {
  for (int i = 0; i < 20; i++) {
    if (reading[i]) {
      int dr = (reading[i] & 1);
      int ar = (reading[i] & 2);

      if (i < 10 && dr) {
        send(0x03, i, digitalRead(i));
      } else {
        if (dr) {
          send(0x03, i, digitalRead(i));
        } else {
          if (ar) {
            send(0x04, i, analogRead(i));
          }
        }
      }
    }
  }
}

char myIpString[32];


void setup() {
  server.begin();
  if (DEBUG)
    Serial.begin(115200);

  IPAddress myIp = Network.localIP();
  sprintf(myIpString, "%d.%d.%d.%d:%d", myIp[0], myIp[1], myIp[2], myIp[3], PORT);
  Spark.variable("endpoint", myIpString, STRING);

}

void loop() {


  if (client.connected()) {
    report();
    while (client.available()) {
      // parse and execute commands

      int action = client.read();
      if (DEBUG)
        Serial.println("Action received: " + ('0' + action));

      int pin, mode, val, type, speed, address, stop, len, i;
      switch (action) {
        case 0x00:  // pinMode
          pin = client.read();
          mode = client.read();
          // mode is modeled after Standard Firmata
          if (mode == 0x00) {
            pinMode(pin, INPUT);
          } else if (mode == 0x02) {
            pinMode(pin, INPUT_PULLUP);
          } else if (mode == 0x03) {
            pinMode(pin, INPUT_PULLDOWN);
          } else if (mode == 0x01) {
            pinMode(pin, OUTPUT);
          }
          break;
        case 0x01:  // digitalWrite
          pin = client.read();
          val = client.read();
          digitalWrite(pin, val);
          break;
        case 0x02:  // analogWrite
          pin = client.read();
          val = client.read();
          analogWrite(pin, val);
          break;
        case 0x03:  // digitalRead
          pin = client.read();
          val = digitalRead(pin);
          server.write(0x03);
          server.write(pin);
          server.write(val);
          break;
        case 0x04:  // analogRead
          pin = client.read();
          val = analogRead(pin);
          server.write(0x04);
          server.write(pin);
          server.write(val);
          break;
        case 0x05: // set always send bit
          pin = client.read();
          val = client.read();
          reading[pin] = val;
          break;

        // Serial API
        case 0x10:  // serial.begin
           type = client.read();
           speed = client.read();
          if (type == 0) {
            Serial.begin(SerialSpeed[speed]);
          } else {
            Serial1.begin(SerialSpeed[speed]);
          }
          break;
        case 0x11:  // serial.end
          type = client.read();
          if (type == 0) {
            Serial.end();
          } else {
            Serial1.end();
          }
          break;
        case 0x12:  // serial.peek
          type = client.read();
          if (type == 0) {
            val = Serial.peek();
          } else {
            val = Serial1.peek();
          }
          server.write(0x07);
          server.write(type);
          server.write(val);
          break;
        case 0x13:  // serial.available()
          type = client.read();
          if (type == 0) {
            val = Serial.available();
          } else {
            val = Serial1.available();
          }
          server.write(0x07);
          server.write(type);
          server.write(val);
          break;
        case 0x14:  // serial.write
          type = client.read();
          len = client.read();
          for (i = 0; i < len; i++) {
            if (type ==0) {
              Serial.write(client.read());
            } else {
              Serial1.write(client.read());
            }
          }
          break;
        case 0x15: // serial.read
          type = client.read();
          if (type == 0) {
            val = Serial.read();
          } else {
            val = Serial1.read();
          }
          server.write(0x16);
          server.write(type);
          server.write(val);
          break;
        case 0x16: // serial.flush
          type = client.read();
          if (type == 0) {
            Serial.flush();
          } else {
            Serial1.flush();
          }
          break;


        // SPI API
        case 0x20:  // SPI.begin
          SPI.begin();
          break;
        case 0x21:  // SPI.end
          SPI.end();
          break;
        case 0x22:  // SPI.setBitOrder
          type = client.read();
          SPI.setBitOrder((type ? MSBFIRST : LSBFIRST));
          break;
        case 0x23:  // SPI.setClockDivider
          val = client.read();
          if (val == 0) {
            SPI.setClockDivider(SPI_CLOCK_DIV2);
          } else if (val == 1) {
            SPI.setClockDivider(SPI_CLOCK_DIV4);
          } else if (val == 2) {
            SPI.setClockDivider(SPI_CLOCK_DIV8);
          } else if (val == 3) {
            SPI.setClockDivider(SPI_CLOCK_DIV16);
          } else if (val == 4) {
            SPI.setClockDivider(SPI_CLOCK_DIV32);
          } else if (val == 5) {
            SPI.setClockDivider(SPI_CLOCK_DIV64);
          } else if (val == 6) {
            SPI.setClockDivider(SPI_CLOCK_DIV128);
          } else if (val == 7) {
            SPI.setClockDivider(SPI_CLOCK_DIV256);
          }
          break;

        case 0x24:  // SPI.setDataMode
          val = client.read();
          if (val == 0) {
            SPI.setDataMode(SPI_MODE0);
          } else if (val == 1) {
            SPI.setDataMode(SPI_MODE1);
          } else if (val == 2) {
            SPI.setDataMode(SPI_MODE2);
          } else if (val == 3) {
            SPI.setDataMode(SPI_MODE3);
          }
          break;

        case 0x25:  // SPI.transfer
          val = client.read();
          val = SPI.transfer(val);
          server.write(0x24);
          server.write(val);
          break;


        // Wire API
        case 0x30:  // Wire.begin
          address = client.read();
          if (address == 0) {
            Wire.begin();
          } else {
            Wire.begin(address);
          }
          break;
        case 0x31:  // Wire.requestFrom
          address = client.read();
          val = client.read();
          stop = client.read();
          Wire.requestFrom(address, val, stop);
          break;
        case 0x32:  // Wire.beginTransmission
          address = client.read();
          Wire.beginTransmission(address);
          break;
        case 0x33:  // Wire.endTransmission
          stop = client.read();
          val = Wire.endTransmission(stop);
          server.write(0x33);
          server.write(val);
          break;
        case 0x34:  // Wire.write
          len = client.read();
          uint8_t wireData[len];
          for (i = 0; i< len; i++) {
            wireData[i] = client.read();
          }
          val = Wire.write(wireData, len);
          server.write(0x34);
          server.write(val);
          break;
        case 0x35:  // Wire.available
          val = Wire.available();
          server.write(0x35);
          server.write(val);
          break;
        case 0x36:  // Wire.read
          val = Wire.read();
          server.write(0x36);
          server.write(val);
          break;


        case 0x40:
          pin = client.read();
          servos[pin].attach(pin);
          break;
        case 0x41:
          pin = client.read();
          val = client.read();
          servos[pin].write(val);
          break;
        case 0x42:
          pin = client.read();
          val = client.read();
          servos[pin].writeMicroseconds(val);
          break;
        case 0x43:
          pin = client.read();
          val = servos[pin].read();
          server.write(0x43);
          server.write(pin);
          server.write(val);
          break;
        case 0x44:
          pin = client.read();
          val = servos[pin].attached();
          server.write(0x44);
          server.write(pin);
          server.write(val);
          break;
        case 0x45:
          pin = client.read();
          servos[pin].detach();
          break;


        default: // noop
          break;

      } // <-- This is the end of the switch
    }
  }
}
