// Initialize left joystick using nipplejs
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
    const x = data.vector.x * 75; // Scale movement to joystick radius
    const y = data.vector.y * 75;
    updateJoystick('left-joystick', x, -y); // Update left joystick
});

// Handle right joystick movement
rightJoystick.on('move', function(evt, data) {
    const x = data.vector.x * 75; // Scale movement to joystick radius
    const y = data.vector.y * 75;
    updateJoystick('right-joystick', x, -y); // Update right joystick
});

// Reset joystick position on release
leftJoystick.on('end', function() {
    updateJoystick('left-joystick', 0, 0); // Reset to center
});

rightJoystick.on('end', function() {
    updateJoystick('right-joystick', 0, 0); // Reset to center
});
