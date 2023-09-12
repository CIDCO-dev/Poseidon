var socket;

function processDiagnostics(diagnostics) {
	console.log(diagnostics);
}

function processRunningNodes(runningNodes){
	console.log(runningNodes);
	// Get a reference to the table container div
	var tableContainer = document.getElementById('table_container');

	// Create an HTML table element
	var table = document.createElement('table');
	table.classList.add('table'); // Add CSS classes as needed

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
	console.log(msg);
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
	var cmd = { command: "updateDiagnostic" };
	socket.send(JSON.stringify(cmd));
}

//******************************
// Main
//******************************

socket = new WebSocket("ws://" + window.location.hostname + ":9099");

socket.onmessage = function (event) {
	var msg = JSON.parse(event.data);
	processMessage(msg);
}

socket.onopen = function (event) {
	updateDiagnostic();
}
