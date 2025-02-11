function processState(state) {
	if (!state || !state.telemetry) {
        
        return;
    }

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
	if (temperatureValue < 75) {
		cputempElement.addClass("bg-gradient-success");
		} else if (temperatureValue >= 75 && temperatureValue <= 80) {
			cputempElement.addClass("bg-gradient-warning");
		} else if (temperatureValue > 80) {
			cputempElement.addClass("bg-gradient-danger");
		}
	$("#cputempText").text(state.telemetry.vitals[0] + "\u00B0" + "C");
	$("#cputemp").width(10 + state.telemetry.vitals[0] + "%");


	// Memory
	$("#freeram").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[2] > 10 ? "bg-gradient-danger" : "bg-gradient-success")).addClass((state.telemetry.vitals[2] > 10 ? "bg-gradient-success" : "bg-gradient-danger"));

	$("#freeramText").text(100 - state.telemetry.vitals[2] + "%");
	$("#freeram").width(100 - state.telemetry.vitals[2] + "%");

	// HDD free space
	if(state.telemetry.vitals[3] <= 5.0){
		$("#freehdd").removeClass("bg-gradient-warning").removeClass("bg-gradient-danger").addClass("bg-gradient-danger");
	}
	else if(state.telemetry.vitals[3] <= 20.0 && state.telemetry.vitals[3] > 5.0){
		$("#freehdd").removeClass("bg-gradient-success").removeClass("bg-gradient-danger").addClass("bg-gradient-warning");
	}
	else{
		$("#freehdd").removeClass("bg-gradient-warning").removeClass("bg-gradient-danger").addClass("bg-gradient-success");
	}
	

	$("#freehddText").text(100 - state.telemetry.vitals[3] + "%");
	$("#freehdd").width(100 - state.telemetry.vitals[3] + "%");
	
	// Humidity
	if (state.telemetry.vitals[7] === -555) {
        $("#humidity").parent().hide();
		$("#humidityText").parent().hide();
    } else {
	$("#humidity").removeClass("bg-gradient-warning").removeClass((state.telemetry.vitals[7] > 40 ? "bg-gradient-success" : "bg-gradient-danger")).addClass((state.telemetry.vitals[7] > 40 ? "bg-gradient-danger" : "bg-gradient-success"));

	$("#humidityText").text(state.telemetry.vitals[7] + "%");
	$("#humidity").width(state.telemetry.vitals[7] + "%");
	}
	// Temperature
	if (state.telemetry.vitals[5] === -555) {
        $("#temperature").parent().hide();
		$("#temperatureText").parent().hide();
    } else {
	var tempElement = $("#temperature");
	var temperatureHBValue = state.telemetry.vitals[5];
	tempElement.removeClass("bg-gradient-success bg-gradient-warning bg-gradient-danger");
	if (temperatureHBValue < 60) {
		tempElement.addClass("bg-gradient-success");
		} else if (temperatureHBValue >= 60 && temperatureHBValue <= 75) {
			tempElement.addClass("bg-gradient-warning");
		} else if (temperatureHBValue > 75) {
			tempElement.addClass("bg-gradient-danger");
		}
	$("#temperatureText").text(state.telemetry.vitals[5] + "\u00B0" + "C");
	$("#temperature").width(20 + state.telemetry.vitals[5] + "%");
	}

	// Battery
	if (state.telemetry.vitals[6] === -555) {
        $("#battery").parent().hide();
		$("#batteryText").parent().hide();
    } else {
	var voltageElement = $("#battery");
	var voltage = state.telemetry.vitals[6];
	voltageElement.removeClass("bg-gradient-success bg-gradient-warning bg-gradient-danger");
	if (voltage <= 11.0) {
		voltageElement.addClass("bg-gradient-danger");
		$("#battery").width(voltage * 4 + "%");
		} else if (voltage >= 11.9 && voltage < 13.0) {
			voltageElement.addClass("bg-gradient-success");
			$("#battery").width(voltage * 4 + "%");
		} else if (voltage > 11.0 && voltage < 11.9) {
			voltageElement.addClass("bg-gradient-warning");
			$("#battery").width(voltage * 4 + "%");
		} else if (voltage > 13.0) {
			voltageElement.addClass("bg-gradient-danger");
			$("#battery").width(voltage * 4 + "%");
		}
	$("#batteryText").text(state.telemetry.vitals[6] + "V");
	}
}

var socket = new WebSocket("ws://" + window.location.hostname + ":9002");

socket.onmessage = function (event) {
	var state = JSON.parse(event.data);
	processState(state);
	//console.log(state);
}
