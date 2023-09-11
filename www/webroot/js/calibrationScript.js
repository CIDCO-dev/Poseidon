function sendZeroImu() {
    var cmd = { "command": "zeroImu" };

    socket.send(JSON.stringify(cmd));
  }

  function processState(state) {

  }

  var socket = new WebSocket("ws://" + window.location.hostname + ":9004");

  socket.onmessage = function (event) {
    var state = JSON.parse(event.data);
    processState(state);
  }
