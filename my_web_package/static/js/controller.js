// controller.js

// Function to update joystick coordinates
function updateJoystick(id, x, y) {
    const joystickContainer = document.getElementById(id);
    const joystickInner = document.querySelector(`#${id} .joystick-inner`);
    
    if (joystickContainer && joystickInner) {
        // Size of the inner joystick
        const joystickSize = 75; // The size of the inner joystick (80px)
        const halfJoystickSize = joystickSize / 2;

        // Calculate the offset
        const maxOffset = 75; // The radius of the inner joystick
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
        const maxOffset = 75;
        const deadZone = 0.15;
        
        const leftStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[0]) > deadZone ? gamepad.axes[0] : 0) * maxOffset));
        const leftStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[1]) > deadZone ? gamepad.axes[1] : 0 ) * maxOffset));
        const rightStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[2]) > deadZone ? gamepad.axes[2] : 0 ) * maxOffset));
        const rightStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[3]) > deadZone ? gamepad.axes[3] : 0 ) * maxOffset));
        
        console.log(`Left Stick X: ${leftStickX}, Y: ${leftStickY}`);
        console.log(`Right Stick X: ${rightStickX}, Y: ${rightStickY}`);
        
        updateJoystick('left-joystick', leftStickX, leftStickY);
        updateJoystick('right-joystick', rightStickX, rightStickY);

        sendJoystickData(leftStickX,leftStickY,rightStickX,rightStickY);
    }
}

// Game loop for continuous input processing
function gameLoop() {
    handleGamepadInput();
    requestAnimationFrame(gameLoop);
}

// Event listener for gamepad connections
window.addEventListener('gamepadconnected', () => {
    gameLoop();
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


window.addEventListener('gamepaddisconnected', () => {
    console.log('Gamepad disconnected.');
});
