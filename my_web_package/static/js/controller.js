let isTouchMode = false;

function updateJoystick(id, x, y) {
    const joystickInner = document.querySelector(`#${id} .joystick-inner`);

    if (joystickInner) {
        const joystickSize = 75;
        const halfJoystickSize = joystickSize / 2;

        const maxOffset = 75;
        const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x));
        const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y));

        joystickInner.style.transform = `translate(${offsetX - halfJoystickSize}px, ${offsetY - halfJoystickSize}px)`;
    }
}

function handleGamepadInput() {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        const gamepad = gamepads[0];

        const maxOffset = 75;
        const deadZone = 0.1;

        const leftStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[0]) > deadZone ? gamepad.axes[0] : 0) * maxOffset));
        const leftStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[1]) > deadZone ? gamepad.axes[1] : 0) * maxOffset));
        const rightStickX = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[2]) > deadZone ? gamepad.axes[2] : 0) * maxOffset));
        const rightStickY = Math.max(-maxOffset, Math.min(maxOffset, (Math.abs(gamepad.axes[3]) > deadZone ? gamepad.axes[3] : 0) * maxOffset));

        updateJoystick('left-joystick', leftStickX, leftStickY);
        updateJoystick('right-joystick', rightStickX, rightStickY);

        const leftTrigger = gamepad.axes[4];
        const rightTrigger = gamepad.axes[5];

        if (!isTouchMode) {
            useControllerData(leftStickX / 75, leftStickY / 75, rightStickX / 75, rightStickY / 75, leftTrigger, rightTrigger, gamepad.buttons);
        }
    }
}

function gameLoop() {
    if (!isTouchMode) {
        handleGamepadInput();
    }
    requestAnimationFrame(gameLoop);
}

window.addEventListener('gamepadconnected', () => {
    gameLoop();
});
