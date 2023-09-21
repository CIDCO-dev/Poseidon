var socket;

function processDiagnostics(diagnostics) {
	console.log('processDiagnostics');
	
	var loadingSpinner = document.getElementById("loading-spinner");
	loadingSpinner.classList.add("d-none");
	
	var tableContainer = document.getElementById('diagnostics');
	
	var diagnosticsTable = document.getElementById('diagnosticsTable');
	if (diagnosticsTable) {
		diagnosticsTable.remove();
	}
	
	var table = document.createElement('table');
	table.id = 'diagnosticsTable';
	table.classList.add('table-responsive', 'table-bordered', 'cell-padding');

	// Create the table header row
	var headerRow = table.insertRow(0);
	var col1 = document.createElement('th');
	col1.textContent = "Status";
	headerRow.appendChild(col1);
	
	var col2 = document.createElement('th');
	col2.textContent = "Diagnostic";
	headerRow.appendChild(col2);
	
	var col3 = document.createElement('th');
	col3.textContent = "Info";
	headerRow.appendChild(col3);
	
	diagnostics.forEach(function (diagnostic, index) {
		var row = table.insertRow(index + 1);
		var cell = row.insertCell(-1);
		cell.textContent = diagnostic.status ? "✅" : "❌"; 
		
		var cell1 = row.insertCell(-1);
		cell1.textContent = diagnostic.name;
		
		var cell2 = row.insertCell(-1);
		cell2.innerHTML = diagnostic.message;
		
	});
	
	tableContainer.appendChild(table);
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
	table.classList.add('table-responsive', 'table-bordered'); // Add CSS classes as needed
	
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


//******************************
// Main
//******************************
socket = new WebSocket("ws://" + window.location.hostname + ":9099");

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
	
	
