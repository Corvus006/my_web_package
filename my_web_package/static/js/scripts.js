var rightX = 0,rightY = 0,leftX = 0,leftY = 0;

function sendJoystickData(leftX, leftY, rightX, rightY) {
    fetch('/joystick_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            left_x: leftX,
            left_y: leftY,
            right_x: rightX,
            right_y: rightY
        }),
    })
    .then(response => response.json())
    .then(data => {
        console.log('Success:', data);
    })
    .catch((error) => {
        console.error('Error:', error);
    });
}