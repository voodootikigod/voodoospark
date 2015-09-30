## What is VoodooSpark?

VoodooSpark is a customized firmware build for [Particle's Spark Core and Photon devices](https://www.particle.io) to allow a remote interface definition of the firmware API over a local TCP connection. The intent is to allow client-side programs to directly control the Particle devices in real-time regardless of their programming language. The interface exposed is directly mapped to the one provided by Particle available in their [docs section](http://docs.particle.io/).

The VoodooSpark uses the [Particle Cloud](http://docs.particle.io/core/api/#introduction) and its REST API to provide IP address and port information to the local Particle device. It will then initiate a direct connection to the host machine, on which will need to be a TCP server. Once the connection has been made, the host machine can drive the Particle devices using the binary protocol defined below to effectively execute firmware API level commands dynamically.

## Loading the Firmware

With your Particle device connected to a Wifi network and has already gone through the ["claim"/ownership process](http://docs.particle.io/core/api/#introduction-claim-device):

1.  Open the [Particle.io Editor](https://build.particle.io/build) with the credentials used when going through the claiming process.
2.  Copy and paste the entire contents of [firmware/voodoospark.cpp](https://raw.githubusercontent.com/voodootikigod/voodoospark/master/firmware/voodoospark.cpp) into the editor window.
3.  Click "Verify"
4.  Click "Flash"
5.  Once the flashing process is complete, close the Particle.io Editor.

Alternately, the firmware may be loaded using the Particle CLI ([particle-cli](https://github.com/spark/particle-cli)) instead of the Particle.io Editor:

``` bash
npm install -g particle-cli
particle cloud login
particle cloud flash PARTICLE_DEVICE_ID firmware/voodoospark.cpp
```

Now your Particle device is running VoodooSpark, lets connect to it!

## Connecting the Particle device to You!

The way VoodooSpark works is to use the Particle Cloud as a channel to identify where to initiate the TCP connection to the device from your host machine.

In order to connect the Particle device to your computer, you will first need to issue an HTTP GET request to the Particle Cloud. This can be done via any programming language, but for this example we are using a simple CURL command. You will need some information outlined with curly braces below, please note the {DEVICE-ID} and {ACCESS-TOKEN} are available from the [Particle.io Editor](https://build.particle.io/build)

    curl https://api.particle.io/v1/devices/{DEVICE-ID}/endpoint?access_token={ACCESS-TOKEN}

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

The "result" value is the IP address of the Particle Device on your local network and the part after the colon (:) is the port that the server is currently listening on. This port will by default be 48879 (0xBEEF), but can be changed in the voodoospark firmware. Please do not hardcode the port for this reason, rather use the data returned back as the response.

With the IP Address and TCP port information, use your favorite language or TCP client to connect to the device (even telnet will work) and send it the necessary BINARY protocol commands to trigger the desired API interactions as defined in our [API Command guide](http://voodoospark.me/#api).


## How to Debug

In case you want to see what is going inside the VoodooSpark in real-time, we have built in a lot of debug hooks for you. You will need a USB cable and the `screen` or `minicom` utilities (one of them) on unix. Modify the firmware loaded in the Particle build system to convert the line:

    #define DEBUG 0

to the following definition:

    #define DEBUG 1

This will enable debug mode, boot a serial port connection on the device and present on your computer for you to watch the inside voodoo. Be sure to flash the new firmware to your device, this is very important and easy to miss. Once the flashing finishes, do an

    ls /dev

Look for something similar to `tty.usbmodem1411` yours may be different, but will be something like tty.usbmodemABCD. Now using your favorite term app connect to that port using the baud rate of 115200.

For screen this command will look like:

    screen /dev/tty.usbmodem1411 115200



## Reference Implementations

*   [Particle-io _node.js_](https://github.com/rwaldron/spark-io)
*   [Vspark _Go_](https://github.com/audreylim/vspark)

## License
See [LICENSE](https://github.com/voodootikigod/voodoospark/blob/master/LICENSE-MIT) file.



## Made With Voodoo

This firmware is made and cared for by the following awesome people:

- Chris Williams https://github.com/voodootikigod
- Rick Waldron https://github.com/rwaldron
- David Resseguie https://github.com/Resseguie
- Brian Genisio https://github.com/BrianGenisio
