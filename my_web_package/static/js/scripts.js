// Initialize left joystick
var leftJoystick = nipplejs.create({
    zone: document.getElementById('left-joystick'),
    mode: 'static',
    position: {left: '50%', top: '50%'},
    color: 'blue'
});

// Initialize right joystick
var rightJoystick = nipplejs.create({
    zone: document.getElementById('right-joystick'),
    mode: 'static',
    position: {left: '50%', top: '50%'},
    color: 'red'
});

// Handle joystick movement
leftJoystick.on('move', function(evt, data) {
    console.log('Left Joystick:', data);
    // Process left joystick data (e.g., send to server)
});

rightJoystick.on('move', function(evt, data) {
    console.log('Right Joystick:', data);
    // Process right joystick data (e.g., send to server)
});

// Gamepad API
window.addEventListener("gamepadconnected", (event) => {
    const gamepad = event.gamepad;
    console.log("Controller connected:", gamepad);

    // Polling loop to read gamepad input
    function pollGamepad() {
        const gp = navigator.getGamepads()[gamepad.index];
        if (gp) {
            console.log("Axes:", gp.axes);
            console.log("Buttons:", gp.buttons);

            // Process gamepad input here
        }
        requestAnimationFrame(pollGamepad);
    }
    pollGamepad();
});
