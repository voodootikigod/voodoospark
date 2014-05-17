/**
  ******************************************************************************
  * @file    voodoospark.cpp
  * @author  Chris Williams
  * @version V2.0.4
  * @date    07-May-2014
  * @brief   Exposes the firmware level API through a TCP Connection initiated
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

#define DEBUG 1

// Port = 0xbeef
#define PORT 48879

// TCPClient client;
TCPServer server = TCPServer(PORT);
TCPClient client;
byte readBuffer[128];
byte reading[20];
byte previous[20];
Servo servos[8];

long SerialSpeed[] = { 600, 1200, 2400, 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200 };


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

char myIpString[24];


void setup() {

  for (int i = 0; i < 20; i++) {
    reading[i] = 0;
    previous[i] = 0;
  }

  server.begin();
  netapp_ipconfig(&ip_config);

  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  IPAddress myIp = Network.localIP();
  sprintf(myIpString, "%d.%d.%d.%d:%d", myIp[0], myIp[1], myIp[2], myIp[3], PORT);
  Spark.variable("endpoint", myIpString, STRING);

}

// table of action codes
// to do: make this an enum?
#define msg_pinMode                    (0x00)
#define msg_digitalWrite               (0x01)
#define msg_analogWrite                (0x02)
#define msg_digitalRead                (0x03)
#define msg_analogRead                 (0x04)
#define msg_setAlwaysSendBit           (0x05)
/* NOTE GAP */
#define msg_serialBegin                (0x10)
#define msg_serialEnd                  (0x11)
#define msg_serialPeek                 (0x12)
#define msg_serialAvailable            (0x13)
#define msg_serialWrite                (0x14)
#define msg_serialRead                 (0x15)
#define msg_serialFlush                (0x16)
/* NOTE GAP */
#define msg_spiBegin                   (0x20)
#define msg_spiEnd                     (0x21)
#define msg_spiSetBitOrder             (0x22)
#define msg_spiSetClockDivider         (0x23)
#define msg_spiSetDataMode             (0x24)
#define msg_spiTransfer                (0x25)
/* NOTE GAP */
#define msg_wireBegin                  (0x30)
#define msg_wireRequestFrom            (0x31)
#define msg_wireBeginTransmission      (0x32)
#define msg_wireEndTransmission        (0x33)
#define msg_wireWrite                  (0x34)
#define msg_wireAvailable              (0x35)
#define msg_wireRead                   (0x36)
/* NOTE GAP */
#define msg_servoAttach                (0x40)
#define msg_servoWrite                 (0x41)
#define msg_servoWriteMicroseconds     (0x42)
#define msg_servoRead                  (0x43)
#define msg_servoAttached              (0x44)
#define msg_servoDetach                (0x45)

#define msg_count                      (0x46)

//
// each position in the array corresponds to an action received from the client.
// e.g. msgMinLength[0] corresponds to action == 0 in the switch(action) in loop().
//      msgMinLength[0] == 2 (one byte each for pin and mode)
//
uint8_t msgMinLength[] = {
  // digital/analog I/O
  2,    // msg_pinMode
  2,    // msg_digitalWrite
  2,    // msg_analogWrite
  1,    // msg_digitalRead
  1,    // msg_analogRead
  2,    // msg_setAlwaysSendBit
  // gap from 0x06-0x0f
  0,    // msg_0x06
  0,    // msg_0x07
  0,    // msg_0x08
  0,    // msg_0x09
  0,    // msg_0x0a
  0,    // msg_0x0b
  0,    // msg_0x0c
  0,    // msg_0x0d
  0,    // msg_0x0e
  0,    // msg_0x0f
  // serial I/O
  2,    // msg_serialBegin
  1,    // msg_serialEnd
  1,    // msg_serialPeek
  1,    // msg_serialAvailable
  2,    // msg_serialWrite  -- variable length message!
  1,    // msg_serialRead
  1,    // msg_serialFlush
  // gap from 0x17-0x1f
  0,    // msg_0x17
  0,    // msg_0x18
  0,    // msg_0x19
  0,    // msg_0x1a
  0,    // msg_0x1b
  0,    // msg_0x1c
  0,    // msg_0x1d
  0,    // msg_0x1e
  0,    // msg_0x1f
  // SPI I/O
  0,    // msg_spiBegin
  0,    // msg_spiEnd
  1,    // msg_spiSetBitOrder
  1,    // msg_spiSetClockDivider
  1,    // msg_spiSetDataMode
  1,    // msg_spiTransfer
  // gap from 0x26-0x2f
  0,    // msg_0x26
  0,    // msg_0x27
  0,    // msg_0x28
  0,    // msg_0x29
  0,    // msg_0x2a
  0,    // msg_0x2b
  0,    // msg_0x2c
  0,    // msg_0x2d
  0,    // msg_0x2e
  0,    // msg_0x2f
  // wire I/O
  1,    // msg_wireBegin
  3,    // msg_wireRequestFrom
  1,    // msg_wireBeginTransmission
  1,    // msg_wireEndTransmission
  1,    // msg_wireWrite  -- variable length message!
  0,    // msg_wireAvailable
  0,    // msg_wireRead
  // gap from 0x37-0x3f
  0,    // msg_0x37
  0,    // msg_0x38
  0,    // msg_0x39
  0,    // msg_0x3a
  0,    // msg_0x3b
  0,    // msg_0x3c
  0,    // msg_0x3d
  0,    // msg_0x3e
  0,    // msg_0x3f
  // servo
  1,    // msg_servoAttach
  2,    // msg_servoWrite
  2,    // msg_servoWriteMicroseconds
  1,    // msg_servoRead
  1,    // msg_servoAttached
  1,    // msg_servoDetach
};


// these are outside loop() so they'll retain their values
//    between calls to loop()
int length = 0;
int idx, action, a;

void loop() {
  if (client.connected()) {
    report();

    a = client.available();
    if (a > 0) {

      #ifdef DEBUG
      Serial.print("Bytes Available: ");
      Serial.println(a, HEX);
      #endif

      // length == current buffer length
      // a == # bytes available
      // if the sum exceeds the size of the buffer, decrease the
      //    # of bytes to read by length
      if (length + a > (int) sizeof readBuffer)
        a = sizeof readBuffer - length;



      #ifdef DEBUG
      Serial.print("Bytes Available: ");
      Serial.println(a, HEX);
      #endif
      // read into the buffer at offset length
      client.read(readBuffer,a);

      // increase length by # of extra bytes read
      length += a;

      // start at beginning of buffer.  should be an action.
      idx = 0;

      // parse and execute commands


      #ifdef DEBUG
      Serial.print("Bytes Available: ");
      Serial.println(a, HEX);
      #endif


      while (idx < length) {
        #ifdef DEBUG
        Serial.print("idx :length");
        Serial.print(idx, HEX);
        Serial.println(length, HEX);
        #endif

        action = readBuffer[idx++];
        #ifdef DEBUG
        Serial.print("Action received: ");
        Serial.println(action, HEX);
        #endif

        // is the action valid?
        if (action <= msg_count) {


          // is there enough data left in the buffer to
          //    process this action?
          // if not, stop and fix
          if (idx + msgMinLength[action] <= length) {


            int pin, mode, val, type, speed, address, stop, len, i;
            switch (action) {
              case msg_pinMode:  // pinMode
                pin = readBuffer[idx++];
                mode = readBuffer[idx++];
                #ifdef DEBUG
                Serial.print("PIN received: ");
                Serial.println(pin, HEX);
                Serial.print("MODE received: ");
                Serial.println(mode, HEX);
                #endif
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

              case msg_digitalWrite:  // digitalWrite
                pin = readBuffer[idx++];
                val = readBuffer[idx++];
                #ifdef DEBUG
                Serial.print("PIN received: ");
                Serial.println(pin, HEX);
                Serial.print("VALUE received: ");
                Serial.println(val, HEX);
                #endif
                digitalWrite(pin, val);
                break;

              case msg_analogWrite:  // analogWrite
                pin = readBuffer[idx++];
                val = readBuffer[idx++];
                #ifdef DEBUG
                Serial.print("PIN received: ");
                Serial.println(pin, HEX);
                Serial.print("VALUE received: ");
                Serial.println(val, HEX);
                #endif
                analogWrite(pin, val);
                break;

              case msg_digitalRead:  // digitalRead
                pin = readBuffer[idx++];
                val = digitalRead(pin);
                #ifdef DEBUG
                Serial.print("PIN received: ");
                Serial.println(pin, HEX);
                Serial.print("VALUE sent: ");
                Serial.println(val, HEX);
                #endif
                server.write(0x03);    // could be (action)
                server.write(pin);
                server.write(val);
                break;

              case msg_analogRead:  // analogRead
                pin = readBuffer[idx++];
                val = analogRead(pin);
                #ifdef DEBUG
                Serial.print("PIN received: ");
                Serial.println(pin, HEX);
                Serial.print("VALUE sent: ");
                Serial.println(val, HEX);
                #endif
                server.write(0x04);    // could be (action)
                server.write(pin);
                server.write(val);
                break;

              case msg_setAlwaysSendBit: // set always send bit
                pin = readBuffer[idx++];
                val = readBuffer[idx++];
                reading[pin] = val;
                break;

              // Serial API
              case msg_serialBegin:  // serial.begin
                type = readBuffer[idx++];
                speed = readBuffer[idx++];
                if (type == 0) {
                  Serial.begin(SerialSpeed[speed]);
                } else {
                  Serial1.begin(SerialSpeed[speed]);
                }
                break;

              case msg_serialEnd:  // serial.end
                type = readBuffer[idx++];
                if (type == 0) {
                  Serial.end();
                } else {
                  Serial1.end();
                }
                break;

              case msg_serialPeek:  // serial.peek
                type = readBuffer[idx++];
                if (type == 0) {
                  val = Serial.peek();
                } else {
                  val = Serial1.peek();
                }
                server.write(0x07);
                server.write(type);
                server.write(val);
                break;

              case msg_serialAvailable:  // serial.available()
                type = readBuffer[idx++];
                if (type == 0) {
                  val = Serial.available();
                } else {
                  val = Serial1.available();
                }
                server.write(0x07);
                server.write(type);
                server.write(val);
                break;

              case msg_serialWrite:  // serial.write
                type = readBuffer[idx++];
                len = readBuffer[idx++];

                if (idx + len < length) {
                  for (i = 0; i < len; i++)
                    if (type == 0) {
                      Serial.write(readBuffer[idx++]);
                    } else {
                      Serial1.write(readBuffer[idx++]);
                    }
                } else {
                  // fix up an incomplete message, putting it at the front
                  //    of the buffer:
                  // the first byte has to be the action value, then type
                  //    then len, and then the remaining bytes get copied.
                  // then bail.
                  readBuffer[0] = action;
                  readBuffer[1] = type;
                  readBuffer[2] = len;
                  memcpy (readBuffer + 3, readBuffer + idx, length - idx);
                  length = 0;
                  return;
                }
                break;

              case msg_serialRead: // serial.read
                type = readBuffer[idx++];
                if (type == 0) {
                  val = Serial.read();
                } else {
                  val = Serial1.read();
                }
                server.write(0x16);
                server.write(type);
                server.write(val);
                break;

              case msg_serialFlush: // serial.flush
                type = readBuffer[idx++];
                if (type == 0) {
                  Serial.flush();
                } else {
                  Serial1.flush();
                }
                break;

              // SPI API
              case msg_spiBegin:  // SPI.begin
                SPI.begin();
                break;

              case msg_spiEnd:  // SPI.end
                SPI.end();
                break;

              case msg_spiSetBitOrder:  // SPI.setBitOrder
                type = readBuffer[idx++];
                SPI.setBitOrder((type ? MSBFIRST : LSBFIRST));
                break;

              case msg_spiSetClockDivider:  // SPI.setClockDivider
                val = readBuffer[idx++];
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

              case msg_spiSetDataMode:  // SPI.setDataMode
                val = readBuffer[idx++];
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

              case msg_spiTransfer:  // SPI.transfer
                val = readBuffer[idx++];
                val = SPI.transfer(val);
                server.write(0x24);
                server.write(val);
                break;

              // Wire API
              case msg_wireBegin:  // Wire.begin
                address = readBuffer[idx++];
                if (address == 0) {
                  Wire.begin();
                } else {
                  Wire.begin(address);
                }
                break;

              case msg_wireRequestFrom:  // Wire.requestFrom
                address = readBuffer[idx++];
                val = readBuffer[idx++];
                stop = readBuffer[idx++];
                Wire.requestFrom(address, val, stop);
                break;

              case msg_wireBeginTransmission:  // Wire.beginTransmission
                address = readBuffer[idx++];
                Wire.beginTransmission(address);
                break;

              case msg_wireEndTransmission:  // Wire.endTransmission
                stop = readBuffer[idx++];
                val = Wire.endTransmission(stop);
                server.write(0x33);    // could be (action)
                server.write(val);
                break;

              case msg_wireWrite:  // Wire.write
                len = readBuffer[idx++];
                // uint8_t wireData[len];

                if (idx + len < length) {
                  // for (i = 0; i< len; i++) {
                  // wireData[i] = readBuffer[idx++];
                  // }
                  // val = Wire.write(wireData, len);

                  // note: byte and uint8_t are both typecast from unsigned char.
                  val = Wire.write((uint8_t *) (readBuffer + idx), len);
                  idx += len;
                  server.write(0x34);    // could be (action)
                  server.write(val);
                } else {
                  // fix up an incomplete message, putting it at the front
                  //    of the buffer:
                  // the first byte has to be the action value, then the len,
                  //    and then remaining bytes get copied.
                  // then bail.
                  readBuffer[0] = action;
                  readBuffer[1] = len;
                  memcpy (readBuffer + 2, readBuffer + idx, length - idx);
                  length = 0;
                  return;
                }
                break;

              case msg_wireAvailable:  // Wire.available
                val = Wire.available();
                server.write(0x35);    // could be (action)
                server.write(val);
                break;

              case msg_wireRead:  // Wire.read
                val = Wire.read();
                server.write(0x36);    // could be (action)
                server.write(val);
                break;


              case msg_servoAttach:
                pin = readBuffer[idx++];
                servos[pin].attach(pin);
                break;

              case msg_servoWrite:
                pin = readBuffer[idx++];
                val = readBuffer[idx++];
                servos[pin].write(val);
                break;

              case msg_servoWriteMicroseconds:
                pin = readBuffer[idx++];
                val = readBuffer[idx++];
                servos[pin].writeMicroseconds(val);
                break;

              case msg_servoRead:
                pin = readBuffer[idx++];
                val = servos[pin].read();
                server.write(0x43);    // could be (action)
                server.write(pin);
                server.write(val);
                break;

              case msg_servoAttached:
                pin = readBuffer[idx++];
                val = servos[pin].attached();
                server.write(0x44);    // could be (action)
                server.write(pin);
                server.write(val);
                break;

              case msg_servoDetach:
                pin = readBuffer[idx++];
                servos[pin].detach();
                break;

              default: // noop
                break;

            } // <-- This is the end of the switch
          } // <-- This is the end of if (idx+msgMinLength[] < length)
          else {
            // fix up an incomplete message, putting it at the front of
            //    the buffer:
            // the first byte has to be the action value, then the
            //    remaining bytes get copied.
            // then bail.
            readBuffer[0] = action;
            memcpy (readBuffer + 1, readBuffer + idx, length - idx);
            length = 0;
            return;
          }
        } // <-- This is the end of the valid action check
        // this is a serious problem, if action isn't valid.
        else {
          // action is bad.
          // move on to the next byte.
          idx++;
        }

      } // <-- This is the end of the while

      if (idx < length) {
        // fix up an incomplete message, putting it at the front of
        //    the buffer:
        // the first byte has to be the action value, then the
        //    remaining bytes get copied.
        readBuffer[0] = action;
        memcpy (readBuffer + 1, readBuffer + idx, length - idx);
        length = 0;
      }

    } // <-- This is the end of the length check
  } else {
    // if no client is yet connected, check for a new connection
    client = server.available();
  }
}
