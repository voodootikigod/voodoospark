var Particle = require("../../particle-io");
var board = new Particle({
    token: process.env.PARTICLE_TOKEN,
    deviceId: process.env.PARTICLE_DEVICE_ID
});


function setup() {
    board.pinMode("D1", board.MODES.SERVO);
    board.pinMode("D0", board.MODES.SERVO);

}

board.on("ready", function() {
    console.log("CONNECTED");

    setup();
    // Analog write to a digital pin for Servos

    // This should just make the continuous servo turn
});


function forward() {
    board.servoWrite("D1", 250);
    board.servoWrite("D0", 80);
}
function backward() {
    board.servoWrite("D1", 80);
    board.servoWrite("D0", 250);
}

function stop() {
    board.servoWrite("D1", 0);
    board.servoWrite("D0", 0);
}
