var leftJoystick = nipplejs.create({
    zone: document.getElementById('left-joystick'),
    size: 150,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: 'transparent'
});

var rightJoystick = nipplejs.create({
    zone: document.getElementById('right-joystick'),
    size: 150,
    mode: 'static',
    position: { left: '50%', top: '50%' },
    color: 'transparent'
});

function updateJoystick(id, x, y) {
    const joystickInner = document.querySelector(`#${id} .joystick-inner`);

    if (joystickInner) {
        const maxOffset = 75; // Maximum joystick offset (radius)
        const halfJoystickSize = 75 / 2;

        const offsetX = Math.max(-maxOffset, Math.min(maxOffset, x));
        const offsetY = Math.max(-maxOffset, Math.min(maxOffset, y));

        joystickInner.style.transform = `translate(${offsetX - halfJoystickSize}px, ${offsetY - halfJoystickSize}px)`;
    }
}

leftJoystick.on('move', function (evt, data) {
    leftX = data.vector.x * 75;
    leftY = data.vector.y * -75;
    updateJoystick('left-joystick', leftX, leftY);
    useJoystickData(leftX/75, leftY/75, rightX/75, rightY/75);
});

rightJoystick.on('move', function (evt, data) {
    rightX = data.vector.x * 75;
    rightY = data.vector.y * -75;
    updateJoystick('right-joystick', rightX, rightY);
    useJoystickData(leftX/75, leftY/75, rightX/75, rightY/75);
});

leftJoystick.on('end', function () {
    updateJoystick('left-joystick', 0, 0);
    leftX = 0;
    leftY = 0;
    useJoystickData(leftX, leftY, rightX, rightY);
});

rightJoystick.on('end', function () {
    updateJoystick('right-joystick', 0, 0);
    rightX = 0;
    rightY = 0;
    useJoystickData(leftX, leftY, rightX, rightY);
});
