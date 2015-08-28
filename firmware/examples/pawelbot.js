var five = require("johnny-five");
var Particle = require("../../particle-io");
var keypress = require('keypress');

var board = new five.Board({
    io: new Particle({
        token: process.env.PARTICLE_TOKEN,
        deviceId: process.env.PARTICLE_DEVICE_ID
    })
});

board.on("ready", function() {

    console.log("Welcome to Sumobot Jr!")
    console.log("Control the bot with the arrow keys, and SPACE to stop.")

    var left_wheel = new five.Servo({
        pin: "D1",
        type: "continuous"
    }).stop();
    var right_wheel = new five.Servo({
        pin: "D0",
        type: "continuous"
    }).stop();
    var led = new five.Led("D7");

    process.stdin.resume();
    process.stdin.setEncoding('utf8');
    process.stdin.setRawMode(true);

    process.stdin.on('keypress', function(ch, key) {

        if (!key) return;


        if (key.name == 'q') {

            console.log('Quitting');
            process.exit();

        } else if (key.name == 'up') {

            console.log('Forward');
            left_wheel.ccw();
            right_wheel.cw();

        } else if (key.name == 'l') {

            led.on();

        } else if (key.name == 'down') {

            console.log('Backward');
            left_wheel.cw();
            right_wheel.ccw();

        } else if (key.name == 'left') {

            console.log('Left');
            left_wheel.ccw();
            right_wheel.ccw();


        } else if (key.name == 'right') {

            console.log('Right');
            left_wheel.cw();
            right_wheel.cw();

        } else if (key.name == 'space') {

            console.log('Stopping');
            left_wheel.stop();
            right_wheel.stop();
            led.off();

        }

    });

});