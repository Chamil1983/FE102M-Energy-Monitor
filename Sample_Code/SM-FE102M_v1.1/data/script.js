var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
// Init web socket when the page loads
window.addEventListener('load', onload);

function onload(event) {
    initWebSocket();
    initConfig();
}

function initConfig() {

       document.getElementById('setCallibration').addEventListener('click', setCallibration);

  }

  function setCallibration(){

        var configdata = {
            vsagthetxt: document.getElementById("vsagthetxt").value,
            Llinegaintxt: document.getElementById("Llinegaintxt").value,
            Llineangtxt: document.getElementById("Llineangtxt").value,
            Nlinegaintxt: document.getElementById("Nlinegaintxt").value,
            Nlineangtxt: document.getElementById("Nlineangtxt").value,
            actstartpowthrtxt: document.getElementById("actstartpowthrtxt").value,
            actnoldpwrthrtxt: document.getElementById("actnoldpwrthrtxt").value,
            reactstartpwrthrtxt: document.getElementById("reactstartpwrthrtxt").value,
            reactnoldpwrthrtxt: document.getElementById("reactnoldpwrthrtxt").value,
            Llinecurrmsgaintxt: document.getElementById("Llinecurrmsgaintxt").value,
            Nlinecurrmsgaintxt: document.getElementById("Nlinecurrmsgaintxt").value,
            voffsettxt: document.getElementById("voffsettxt").value,
            Llinecuroffsettxt: document.getElementById("Llinecuroffsettxt").value,
            Nlinecuroffsettxt: document.getElementById("Nlinecuroffsettxt").value,
            Llineactpwtoffsettxt: document.getElementById("Llineactpwtoffsettxt").value,
            Llinereactpwroffsettxt: document.getElementById("Llinereactpwroffsettxt").value,
            Nlineactpwroffsettxt: document.getElementById("Nlineactpwroffsettxt").value,
            Nlinereactpwroffsettxt: document.getElementById("Nlinereactpwroffsettxt").value,

            crc1txt: document.getElementById("crc1txt").value,
            crc2txt: document.getElementById("crc2txt").value


        };
        websocket.send(JSON.stringify(configdata));


  }


function getSystemInfo(){
    websocket.send("getmeterInfo");
}

function getConfigInfo(){
    websocket.send("getmeterconfig");
}

function getMtrReadings(){
    websocket.send("getreadings");
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

// When websocket is established, call the getSystemInfo() function
function onOpen(event) {
    console.log('Connection opened');
    getSystemInfo();
    getConfigInfo();
    getMtrReadings();
}

function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
}

// Function that receives the message from the ESP32 with the readings
function onMessage(event) {
    console.log(event.data);
    var myObj = JSON.parse(event.data);
    var keys = Object.keys(myObj);

    for (var i = 0; i < keys.length; i++){
        var key = keys[i];
        document.getElementById(key).innerHTML = myObj[key];
    }
}

