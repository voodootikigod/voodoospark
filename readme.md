## What is VoodooSpark?

VoodooSpark is a customized firmware build for the [Spark Core device](https://www.spark.io) to allow a remote interface definition of the firmware API over a local TCP connection. The intent is to allow client-side programs to directly control the Spark Core in real-time regardless of their programming language. The interface exposed is directly mapped to the one provided by Spark available in their [docs section](https://docs.spark.io/).

The VoodooSpark uses the [Spark Cloud](http://docs.spark.io/#/start/wait-what-is-this-thing-the-spark-cloud) and its REST API to provide IP address and port information to the local spark core. It will then initiate a direct connection to the host machine, on which will need to be a TCP server. Once the connection has been made, the host machine can drive the Spark Core using the binary protocol defined below to effectively execute firmware API level commands dynamically.

## Loading the Firmware

With your Spark device connected to a Wifi network and has already gone through the ["claim"/ownership process](http://docs.spark.io/#/start/step-1-power-the-core):

1.  Open the [Spark.io Editor](https://www.spark.io/build) with the credentials used when going through the claiming process.
2.  Copy and paste the entire contents of [src/voodoospark.cpp](https://raw.githubusercontent.com/voodootikigod/voodoospark/master/src/voodoospark.cpp) into the editor window.
3.  Click "Verify"
4.  Click "Flash"
5.  Once the flashing process is complete, close the Spark.io Editor.

Now your Spark Core is running VoodooSpark, lets connect to it!

## Connecting the Spark Core to You!

The way VoodooSpark works is to use the Spark Cloud as a channel to identify where to initiate the TCP connection to the Spark Device from your host machine.

In order to connect the Spark Core to your computer, you will first need to issue an HTTP GET request to the Spark Cloud. This can be done via any programming language, but for this example we are using a simple CURL command. You will need some information outlined with curly braces below, please note the {DEVICE-ID} and {ACCESS-TOKEN} are available from the [Spark.io Editor](https://www.spark.io/build)

    curl https://api.spark.io/v1/devices/{DEVICE-ID}/endpoint?access_token={ACCESS-TOKEN}"

This should return a JSON document that looks similar to this:

    {
      "cmd": "VarReturn",
      "name": "endpoint",
      "result": "192.168.1.10:48879",
      "coreInfo": {
        "last_app": "",
        "last_heard": "2014-05-08T02:51:48.826Z",
        "connected": true,
        "deviceID": "{DEVICE-ID}"
      }
    }

The result value is the IP address of the Spark Device on your local network and the part after the color (:) is the port that the server is currently listening on. This port will by default be 48879 (0xBEEF), but can be changed in the voodoospark firmware. Please do not hardcode the port for this reason, rather use the data returned back as the response.

With the IP Address and TCP port information, use your favorite language or TCP client to connect to the device (even telnet will work) and send it the necessary BINARY protocol commands to trigger the desired API interactions as defined in our [API Command guide](/api.html).

## Reference Implementations

*   [Spark-io _node.js_](http://github.com/rwaldron/spark-io)
# VoodooSpark 2.0.0

VoodooSpark is a special case firmware build for the [Spark Core device](https://www.spark.io) to allow a remote interface definition across a wireless connection. The intent is to allow client side programs to directly control the Spark Core in real-time regardless of their programming language. The interface exposed is directly mapped to the one provided by Spark available in their [docs section](https://docs.spark.io/).

__This README and project are a derivative from the [spark-io](https://github.com/rwaldron/spark-io) project by [Rick Waldron](https://github.com/rwaldron). I have extracted the firmware that I authored to allow it to be used by other projects and languages and be able to be updated independently based on Spark API changes.__

### Getting Started

With your Spark device connected to a Wifi network and has already gone through the ["claim"/ownership process](http://docs.spark.io/#/start/step-1-power-the-core):

1. Open the [Spark.io Editor](https://www.spark.io/build) with the credentials used when going through the claiming process.
2. Copy and paste the entire contents of `src/voodoospark.cpp` into the editor window.
3. Click "Verify"
4. Click "Flash"

Once the flashing process is complete, close the Spark.io Editor.

### Blink an Led

The "Hello World" of microcontroller programming:

```js
var Spark = require("spark-io");
var board = new Spark({
  token: "{{yours}}",
  deviceId: "{{yours}}"
});

board.on("ready", function(data) {
  console.log("CONNECTED", data);

  var byte = 0;

  setInterval(function() {
    console.log("message");
    this.digitalWrite("D7", (byte ^= 1));
  }.bind(this), 500);
});
```

### Johnny-Five IO Plugin

Spark-IO can be used as an [IO Plugin](https://github.com/rwaldron/johnny-five/wiki/IO-Plugins) for [Johnny-Five](https://github.com/rwaldron/johnny-five):

```js
var five = require("johnny-five");
var Spark = require("spark-io");
var board = new five.Board({
  io: new Spark({
    token: "{{yours}}",
    deviceId: "{{yours}}"
  })
});

board.on("ready", function() {
  var led = new five.Led("D7");
  led.blink();
});
```


### API

This is copied directly from [The Tinker API](http://docs.spark.io/#/start/tinkering-with-tinker-the-tinker-api).


**digitalWrite(pin, value)**

> Sets the pin to 1 or 0, which either connects it to 3.3V (the maximum voltage of the system) or to GND (ground). Pin D7 is connected to an on-board LED; if you set pin D7 to HIGH, the LED will turn on, and if you set it to LOW, it will turn off.

> The parameters must be the pin (A0 to A7, D0 to D7), followed by either HIGH or LOW, separated by a comma. The return value will be 1 if the write succeeds, and -1 if it fails.

> [Tinker API: digitalWrite](http://docs.spark.io/#/start/the-tinker-api-digitalwrite)

Example:
```js
// This will turn on the on-board LED
board.digitalWrite("D7", 1);
```



**analogWrite(pin, value)**

> Sets the pin to a value between 0 and 255, where 0 is the same as LOW and 255 is the same as HIGH. This is sort of like sending a voltage between 0 and 3.3V, but since this is a digital system, it uses a mechanism called Pulse Width Modulation, or PWM. You could use analogWrite to dim an LED, as an example.

> The parameters must be the pin (A0 to A7, D0 to D7), followed by an integer value from 0 to 255, separated by a comma. The return value will be 1 if the write succeeds, and -1 if it fails.

> [Tinker API: analogWrite](http://docs.spark.io/#/start/the-tinker-api-analogwrite)

Example:
```js
// Crank an LED to full brightness
board.analogWrite("A7", 255);
```

**servoWrite(pin, value)** This is an alias to `analogWrite`


**digitalRead(pin, handler)** Setup a continuous read handler for specific digital pin.

> This will read the digital value of a pin, which can be read as either HIGH or LOW. If you were to connect the pin to 3.3V, it would read HIGH; if you connect it to GND, it would read LOW. Anywhere in between, it’ll probably read whichever one it’s closer to, but it gets dicey in the middle.

> The parameters must be the pin (A0 to A7, D0 to D7). The return value will be between 0 and 4095 if the read succeeds, and -1 if it fails.

> [Tinker API: digitalRead](http://docs.spark.io/#/start/the-tinker-api-digitalread)

Example:
```js
// Log all the readings for D1
board.digitalRead("D1", function(data) {
  console.log(data);
});
```


**analogRead(pin, handler)** Setup a continuous read handler for specific analog pin.

> This will read the analog value of a pin, which is a value from 0 to 4095, where 0 is LOW (GND) and 4095 is HIGH (3.3V). All of the analog pins (A0 to A7) can handle this. analogRead is great for reading data from sensors.

> The parameters must be the pin (A0 to A7, D0 to D7). The return value will be between 0 and 4095 if the read succeeds, and -1 if it fails.

> [Tinker API: analogRead](http://docs.spark.io/#/start/the-tinker-api-analogread)

Example:
```js
// Log all the readings for A1
board.analogRead("A1", function(data) {
  console.log(data);
});

```

## License
See LICENSE file.

