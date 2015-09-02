var five = require("../../johnny-five");
var Particle = require("../../particle-io");
var board = new five.Board({
    io: new Particle({
        token: process.env.PARTICLE_TOKEN,
        deviceId: process.env.PARTICLE_DEVICE_ID
    })
});

board.on("ready", function() {
    var sc = new five.Servo({
        pin: "D1",
        type: "continuous"
    });
    sc.cw();
});