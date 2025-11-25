function sendZeroImu() {
    var cmd = { "command": "zeroImu" };

    socket.send(JSON.stringify(cmd));
  }

  function processState(state) {

  }

  var socket = (typeof WebSocket !== "undefined")
    ? new WebSocket("ws://" + window.location.hostname + ":9004")
    : { send: function () {}, onmessage: null };

  socket.onmessage = function (event) {
    var state = JSON.parse(event.data);
    processState(state);
  }
