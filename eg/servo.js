var Spark = require("../../spark-io");
var board = new Spark({
    token: process.env.SPARK_TOKEN,
    deviceId: process.env.SPARK_DEVICE_ID
});


function setup() {
    board.pinMode("D1", board.MODES.OUTPUT);
    board.pinMode("D0", board.MODES.OUTPUT);

}

board.on("ready", function() {
    console.log("CONNECTED");

    setup();
    // Analog write to a digital pin for Servos

    // This should just make the continuous servo turn
});


function forward() {
    board.analogWrite("D1", 250);
    board.analogWrite("D0", 80);
}
function backward() {
    board.analogWrite("D1", 80);
    board.analogWrite("D0", 250);
}

function stop() {
    board.analogWrite("D1", 0);
    board.analogWrite("D0", 0);
  }
