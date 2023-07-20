function processState(state) {

    // Vitals bars

    // Uptime
    var timeup = state.telemetry.vitals[4];
    var jourup = Math.floor(timeup / 86400);
    var heureup = Math.floor((timeup - (jourup * 86400)) / 3600);
    var minuteup = Math.floor((timeup - (jourup * 86400 + heureup * 3600)) / 60);
    var secondeup = Math.floor(timeup - ((jourup * 86400) + (heureup * 3600) + (minuteup * 60)));
    var uptime = jourup + " ";
    if (heureup < 10) { uptime = uptime + "0" }
    uptime = uptime + heureup + ":";
    if (minuteup < 10) { uptime = uptime + "0" }
    uptime = uptime + minuteup + ":";
    if (secondeup < 10) { uptime = uptime + "0" }
    uptime = uptime + secondeup;
    $("#uptimeText").text(uptime);


    // CPU load
    $("#cpuload").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[1] < 90 ? "bg-gradient-danger" : "bg-gradient-success")).addClass((state.telemetry.vitals[1] < 90 ? "bg-gradient-success" : "bg-gradient-danger"));

    $("#cpuloadText").text(state.telemetry.vitals[1] + "%");
    $("#cpuload").width(state.telemetry.vitals[1] + "%");


    // CPU temperature
    $("#cputemp").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[0] > 60 ? "bg-gradient-success" : "bg-gradient-danger")).addClass((state.telemetry.vitals[0] > 75 ? "bg-gradient-danger" : "bg-gradient-success"));

    $("#cputempText").text(state.telemetry.vitals[0] + "\u00B0" + "C");
    $("#cputemp").width(20 + state.telemetry.vitals[0] + "%");


    // Memory
    $("#freeram").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[2] > 10 ? "bg-gradient-danger" : "bg-gradient-success")).addClass((state.telemetry.vitals[2] > 10 ? "bg-gradient-success" : "bg-gradient-danger"));

    $("#freeramText").text(100 - state.telemetry.vitals[2] + "%");
    $("#freeram").width(100 - state.telemetry.vitals[2] + "%");

    // HDD free space
    $("#freehdd").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[3] > 20 ? "bg-gradient-danger" : "bg-gradient-success")).addClass((state.telemetry.vitals[3] > 20 ? "bg-gradient-success" : "bg-gradient-danger"));

    $("#freehddText").text(100 - state.telemetry.vitals[3] + "%");
    $("#freehdd").width(100 - state.telemetry.vitals[3] + "%");
}

var socket = new WebSocket("ws://" + window.location.hostname + ":9002");

socket.onmessage = function (event) {
    var state = JSON.parse(event.data);
    processState(state);
}