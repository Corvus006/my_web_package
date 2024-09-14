//joystick.js

var leftJoystick = nipplejs.create({
    zone: document.getElementById('left-joystick'),
    size: 150,
    mode: 'static',
    position: {left: '50%', top: '50%'},
    color: 'transparent'
});

// Initialize right joystick using nipplejs
var rightJoystick = nipplejs.create({
    zone: document.getElementById('right-joystick'),
    size: 150,
    mode: 'static',
    position: {left: '50%', top: '50%'},
    color: 'transparent'
});

// Update joystick-inner based on nipplejs movement
function updateJoystick(id, x, y) {
    const joystickInner = document.querySelector(`#${id} .joystick-inner`);
    
    if (joystickInner) {
        const maxOffset = 75; // Maximum joystick offset (radius)
        const halfJoystickSize = 75 / 2; // Size of the joystick inner (75px)

        // Apply limits to the joystick movement
        const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x));
        const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y));

        // Update the joystick-inner position
        joystickInner.style.transform = `translate(${offsetX - halfJoystickSize}px, ${offsetY - halfJoystickSize}px)`;
    }
}

// Handle left joystick movement
leftJoystick.on('move', function(evt, data) {
    leftX = data.vector.x * 75; // Scale movement to joystick radius
    leftY = data.vector.y * -75;
    updateJoystick('left-joystick', leftX, leftY); // Update left joystick
    sendJoystickData(leftX,leftY,rightX,rightY);
});

// Handle right joystick movement
rightJoystick.on('move', function(evt, data) {
    rightX = data.vector.x * 75; // Scale movement to joystick radius
    rightY = data.vector.y * -75;
    updateJoystick('right-joystick', rightX, rightY); // Update right joystick
    sendJoystickData(leftX,leftY,rightX,rightY);
});

// Reset joystick position on release
leftJoystick.on('end', function() {
    updateJoystick('left-joystick', 0, 0); // Reset to center
    leftX = 0;
    leftY = 0;
    sendJoystickData(leftX,leftY,rightX,rightY);
});

rightJoystick.on('end', function() {
    updateJoystick('right-joystick', 0, 0); // Reset to center
    rightX = 0;
    rightY = 0;
    sendJoystickData(leftX,leftY,rightX,rightY);
});
