window.addEventListener('keydown', handleKeyPress);
window.addEventListener('keyup', handleKeyRelease);


function handleKeyPress(event) {
    switch(event.key) {
        case 'W':
        case 'w':
            linearX = 1;
            break;
        case 'S':
        case 's':
            linearX = -1;
            break;
        case 'A':
        case 'a':
            linearY = 1;
            break;
        case 'D':
        case 'd':
            linearY = -1;
            break;
        case 'Q':
        case 'q':
            angularZ = 1;
            break;
        case 'E':
        case 'e':
            angularZ = -1;
            break;
    }
    sendMessage();
}

function handleKeyRelease(event) {
    switch(event.key) {
        case 'W':
        case 'w':
        case 'S':
        case 's':
            linearX = 0;
            break;
        case 'A':
        case 'a':
        case 'D':
        case 'd':
            linearY = 0;
            break;
        case 'Q':
        case 'q':
        case 'E':
        case 'e':
            angularZ = 0;
            break;
    }
    sendMessage();
}
