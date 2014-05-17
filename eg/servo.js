var Spark = require("../../spark-io");
var board = new Spark({
  token: process.env.SPARK_TOKEN,
  deviceId: process.env.SPARK_DEVICE_ID
});

board.on("ready", function() {
  console.log("CONNECTED");

  this.pinMode("D1", this.MODES.OUTPUT);
  // Analog write to a digital pin for Servos
  this.analogWrite("D1", 150);

  // This should just make the continuous servo turn
});