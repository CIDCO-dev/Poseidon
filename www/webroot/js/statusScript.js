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
	var cputempElement = $("#cputemp");
	var temperatureValue = state.telemetry.vitals[0];
	cputempElement.removeClass("bg-gradient-success bg-gradient-warning bg-gradient-danger");
	if (temperatureValue < 60) {
		cputempElement.addClass("bg-gradient-success");
		} else if (temperatureValue >= 60 && temperatureValue <= 75) {
			cputempElement.addClass("bg-gradient-warning");
		} else if (temperatureValue > 75) {
			cputempElement.addClass("bg-gradient-danger");
		}
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
	
	// Humidity
	$("#humidity").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[10] > 40 ? "bg-gradient-success" : "bg-gradient-danger")).addClass((state.telemetry.vitals[10] > 40 ? "bg-gradient-danger" : "bg-gradient-success"));

	$("#humidityText").text(state.telemetry.vitals[10] + "%");
	$("#humidity").width(state.telemetry.vitals[10] + "%");
	
	// Temperature
	var tempElement = $("#temperature");
	var temperatureHBValue = state.telemetry.vitals[7];
	tempElement.removeClass("bg-gradient-success bg-gradient-warning bg-gradient-danger");
	if (temperatureHBValue < 60) {
		tempElement.addClass("bg-gradient-success");
		} else if (temperatureHBValue >= 60 && temperatureHBValue <= 75) {
			tempElement.addClass("bg-gradient-warning");
		} else if (temperatureHBValue > 75) {
			tempElement.addClass("bg-gradient-danger");
		}
	$("#temperatureText").text(state.telemetry.vitals[7] + "\u00B0" + "C");
	$("#temperature").width(20 + state.telemetry.vitals[7] + "%");
	
	// Battery
	var voltageElement = $("#battery");
	var voltage = state.telemetry.vitals[9];
	voltageElement.removeClass("bg-gradient-success bg-gradient-warning bg-gradient-danger");
	if (voltage <= 11) {
		voltageElement.addClass("bg-gradient-danger");
		$("#battery").width(1 + "%");
		} else if (voltage >= 11.5 && voltage <= 12.5) {
			voltageElement.addClass("bg-gradient-success");
			$("#battery").width(99 + "%");
		} else if (voltage > 11 && voltage <= 11.5) {
			voltageElement.addClass("bg-gradient-warning");
			$("#battery").width(50 + "%");
		} else if (voltage > 12.5) {
			voltageElement.addClass("bg-gradient-danger");
			$("#battery").width(100 + "%");
		}
	$("#batteryText").text(state.telemetry.vitals[9] + "V");
	
}

var socket = new WebSocket("ws://" + window.location.hostname + ":9002");

socket.onmessage = function (event) {
	var state = JSON.parse(event.data);
	processState(state);
	//console.log(state);
}
