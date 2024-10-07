
var rightX = 0, rightY = 0, leftX = 0, leftY = 0, image_index = 0;
var enable = false, disable = false;
const speedMulti = 1;
var linearX = 0, linearY = 0, angularZ = 0;

document.getElementById('activate-btn').addEventListener('click', () => {
    enable = true;
    sendMessage();
    enable = false;
});

document.getElementById('deactivate-btn').addEventListener('click', () => {
    disable = true;
    sendMessage();
    disable = false;
});

document.getElementById('topic-select').addEventListener('change', (event) => {
    image_index = parseInt(event.target.value);
    sendMessage();
});

function useJoystickData(leftX, leftY, rightX, rightY) {
    linearX = -leftY * speedMulti;
    linearY = -leftX * speedMulti;
    angularZ = rightX * speedMulti;
    sendMessage();
}

function useControllerData(leftX, leftY, rightX, rightY, leftTrigger, rightTrigger, buttons) {
    enable = buttons[0].pressed;
    disable = buttons[1].pressed;

    let speed = (rightTrigger + 1) / 2 * speedMulti;
    linearY = -leftX * speed;
    linearX = -leftY * speed;
    angularZ = rightX * speed;

    console.log("speed: ", speed);
    console.log("linearX: ", linearX);
    sendMessage();
}

function sendMessage() {
    fetch('/control_data', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json'
        },
        body: JSON.stringify({
            linear_x: linearX,
            linear_y: linearY,
            angular_z: angularZ,
            enable: enable,
            disable: disable,
            image_index:image_index
        }),
    })
    .then(response => response.json())
    .then(data => {
        console.log('Response:', data);
    })
    .catch((error) => {
        console.error('Error:', error);
    });

    // Reset flags
    enable = false;
    disable = false;
}