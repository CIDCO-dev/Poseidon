var socket;

function processDiagnostics(diagnostics) {
	console.log('processDiagnostics');
	
	var loadingSpinner = document.getElementById("loading-spinner");
	loadingSpinner.classList.add("d-none");
	
	//var tableContainer = document.getElementById('diagnostics');
	
	var table = document.getElementById('diagnosticsTable');

	var rowCount = table.rows.length;
	for (var i = rowCount - 1; i > 0; i--) {
		table.deleteRow(i);
	}
	
	diagnostics.forEach(function (diagnostic, index) {
		var row = table.insertRow(index + 1);
		var cell = row.insertCell(-1);
		cell.innerHTML = diagnostic.status ? "✅" : "❌"; 
		cell.style.textAlign = "center";
		
		var cell1 = row.insertCell(-1);
		cell1.textContent = diagnostic.name;
		
		var cell2 = row.insertCell(-1);
		cell2.innerHTML = diagnostic.message.replace(/\n/g, '<br>');
		
	});
	
	//tableContainer.appendChild(table);
}

function processRunningNodes(runningNodes){
	console.log("processRunningNodes");
	
	var tableContainer = document.getElementById('runningNodes');
	
	var runningNodesTable = document.getElementById("runningNodesTable");
	if (runningNodesTable) {
		runningNodesTable.remove();
	}
	
	var table = document.createElement('table');
	table.id = "runningNodesTable";
	table.classList.add('table','table-bordered', 'dataTable');
	
	// Create the table header row
	var headerRow = table.insertRow(0);
	var th = document.createElement('th');
	th.textContent = "Running nodes";
	headerRow.appendChild(th);
	var nodes = runningNodes;

	// Create table rows for the JSON data
	nodes.forEach(function (node, index) {
		var row = table.insertRow(index + 1); // Start from index 1 to skip header row
		var cell = row.insertCell(-1);
		cell.textContent = node;
	});

	// Append the table to the table container
	tableContainer.appendChild(table);
}

function processMessage(msg) {
	if (msg.running_nodes) {
		processRunningNodes(msg.running_nodes);
	}
	if (msg.diagnostics) {
		processDiagnostics(msg.diagnostics);
	}
}


//******************************
//Function to get diagnostics results
//******************************
function updateDiagnostic() {
	
	var loadingSpinner = document.getElementById("loading-spinner");
	loadingSpinner.classList.remove("d-none");
	
	var cmd = { command: "updateDiagnostic" };
	socket.send(JSON.stringify(cmd));
}

function getRunningNodes() {
	var cmd = { command: "getRunningNodes" };
	socket.send(JSON.stringify(cmd));
}

function toggleRecording() {
  if ($("#btnRecording").hasClass("btn-success")) {
    var cmd = { command: "startLogging" };
    socket.send(JSON.stringify(cmd));
  } else if ($("#btnRecording").hasClass("btn-danger")) {
    var cmd = { command: "stopLogging" };
    socket.send(JSON.stringify(cmd));
  }
}
// Functions that changes the color and display status of recording buttons
function showRecording() {
  $("#recordingStatus2").removeClass("d-none text-success").addClass("text-danger");
  $("#recordingStatus2").text("Active");
  // $("#modeWidget").removeClass("text-danger text-success").addClass("text-danger");
}

function showNotRecording() {
  $("#recordingStatus2").removeClass("d-none text-danger").addClass("text-success");
  $("#recordingStatus2").text("Inactive");
  // $("#modeWidget").removeClass("text-danger text-success").addClass("text-success");
}

function showRecordingButton() {
  $("#btnRecording").removeClass("btn-success").addClass("btn-danger");
  $("#btnRecordingText").text("Stop Recording");
  $("#RecIcon").removeClass("text-success").addClass("text-danger");
}

function showNotRecordingButton() {
  $("#btnRecording").removeClass("btn-danger").addClass("btn-success");
  $("#btnRecordingText").text("Start Recording");
  $("#RecIcon").removeClass("text-danger").addClass("text-success");
}

function hideLoggingButton() {
  $("#btnRecording").removeClass("d-inline").addClass("d-none");
  $("#RecIcon").removeClass("d-inline").addClass("d-none");
}

// Resize recording button 
if ($(window).width() < 766) {
  $("#btnRecording").toggleClass("d-inline btn-lg");
  $("#btnRecording").parent().toggleClass("m-0 p-0");
  $("#btnRecording").parent().parent().toggleClass("m-0 p-0");
  $("#btnRecording").css('width', '100%');
  $("#btnRecording").css('height', '100%');
  $("#btnRecording").parent().css('width', '100%');
  $("#btnRecording").parent().css('height', '100%');
}

//******************************
// Main
//******************************
socket = new WebSocket("ws://" + window.location.hostname + ":9099");
// socket = new WebSocket("ws://" + window.location.hostname + ":9002");

socket.onmessage = function (event) {
	var msg = JSON.parse(event.data);
	processMessage(msg);
}

socket.onopen = function (event) {
	getRunningNodes();
	updateDiagnostic();
}
	
	var diagnosticsButton = document.getElementById("diagnosticsButton");
	diagnosticsButton.addEventListener("click",function(){
		getRunningNodes();
		updateDiagnostic();
	});
	
	
