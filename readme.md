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

    curl https://api.spark.io/v1/devices/{DEVICE-ID}/endpoint?access_token={ACCESS-TOKEN}

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

With the IP Address and TCP port information, use your favorite language or TCP client to connect to the device (even telnet will work) and send it the necessary BINARY protocol commands to trigger the desired API interactions as defined in our [API Command guide](http://voodootikigod.github.io/voodoospark/api.html).

## Reference Implementations

*   [Spark-io _node.js_](http://github.com/rwaldron/spark-io)

## License
See [LICENSE](https://github.com/voodootikigod/voodoospark/blob/master/LICENSE-MIT) file.

