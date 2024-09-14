// controller.js

// Function to update joystick coordinates
function updateJoystick(id, x, y) {
    const joystickContainer = document.getElementById(id);
    const joystickInner = document.querySelector(`#${id} .joystick-inner`);
    
    if (joystickContainer && joystickInner) {
        // Size of the inner joystick
        const joystickSize = 80; // The size of the inner joystick (80px)
        const halfJoystickSize = joystickSize / 2;

        // Calculate the offset
        const maxOffset = 80; // The radius of the inner joystick
        const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x));
        const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y));

        // Set the new position of the inner joystick
        joystickInner.style.transform = `translate(${offsetX - halfJoystickSize}px, ${offsetY - halfJoystickSize}px)`;
    }
}

// Function to process gamepad input
function handleGamepadInput() {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        const gamepad = gamepads[0];
        const maxOffset = 80;
        const deadZone = 0.1;
        
        const leftStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[0]) > deadZone ? gamepad.axes[0] : 0) * maxOffset));
        const leftStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[1]) > deadZone ? gamepad.axes[1] : 0 ) * maxOffset));
        const rightStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[2]) > deadZone ? gamepad.axes[2] : 0 ) * maxOffset));
        const rightStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[3]) > deadZone ? gamepad.axes[3] : 0 ) * maxOffset));
        
        console.log(`Left Stick X: ${leftStickX}, Y: ${leftStickY}`);
        console.log(`Right Stick X: ${rightStickX}, Y: ${rightStickY}`);
        
        updateJoystick('left-joystick', leftStickX, leftStickY);
        updateJoystick('right-joystick', rightStickX, rightStickY);
    }
}

// Game loop for continuous input processing
function gameLoop() {
    handleGamepadInput();
    requestAnimationFrame(gameLoop);
}

// Event listener for gamepad connections
window.addEventListener('gamepadconnected', () => {
    console.log('Gamepad connected!');
    gameLoop();
});

window.addEventListener('gamepaddisconnected', () => {
    console.log('Gamepad disconnected.');
});
