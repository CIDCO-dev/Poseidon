var socket;
if (typeof $ !== "undefined") {
$(document).ready(function () {
    socket = (typeof WebSocket !== "undefined")
        ? new WebSocket("ws://" + window.location.hostname + ":9002")
        : { send: function () {} };

    socket.onmessage = function (event) {
        //console.log(event.data);
        var msg = JSON.parse(event.data);

        if (msg.recordingInfo) {
            var isLogging = msg.recordingInfo.status;
            var mode = msg.loggingMode.the_mode_is;
            processRecordingInfo(isLogging, mode);
        }

    };

    socket.onopen = function (event) {
        //init display
        getLoggingInfo();
    };

    // Displays error info, reconnection is handled by the timer
    socket.onerror = function (event) {
        console.error("Websocket error: ", event)
    }

    // Displays closing info, reconnection is handled by the timer
    socket.onclose = function (event) {
        console.log('WebSocket connection closed.');
        console.log('Close code:', event.code);
        console.log('Close reason:', event.reason);
    }
});
}

function getLoggingInfo() {
    var cmd = { command: "getLoggingInfo" };
    if (socket && socket.send) {
        socket.send(JSON.stringify(cmd));
    }
}

function processRecordingInfo(isLogging, mode) {
    //console.log(isLogging);
    //console.log(mode);
	if (mode == "1") {
        hideLoggingButton();
        $("#modeWidget").text("always ON"); // text - success
    }
    else if (mode == "2") {
        $("#modeWidget").text("Manual");
        if (isLogging) {
            //console.log("logging mode 2 : manual , isLogging");
            showRecordingButton();
        }
        else {
            //console.log("logging mode 2 : manual , is not logging");
            showNotRecordingButton();
        }
    }
    else if (mode == "3") {
        hideLoggingButton();
        $("#modeWidget").text("speed based");
    }
}

function toggleRecording() {
    if ($("#RecIcon").hasClass("text-success")) {
        var cmd = { command: "startLogging" };
        if (socket && socket.send) {
            socket.send(JSON.stringify(cmd));
        }
    } else if ($("#RecIcon").hasClass("text-danger")) {
        var cmd = { command: "stopLogging" };
        if (socket && socket.send) {
            socket.send(JSON.stringify(cmd));
        }
    }
}

function showRecordingButton() {
    $("#RecIcon").removeClass("text-success").addClass("text-danger");
}
function showNotRecordingButton() {
    $("#RecIcon").removeClass("text-danger").addClass("text-success");
}
function hideLoggingButton() {
    $("#RecIcon").removeClass("d-inline").addClass("d-none");
    $("#RecIcon").removeClass("d-inline").addClass("d-none");
}
