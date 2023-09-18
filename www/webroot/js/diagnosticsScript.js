var socket;

function processDiagnostics(diagnostics) {
	console.log('processDiagnostics');
	var tableContainer = document.getElementById('diagnostics');
	
	var diagnosticsTable = document.getElementById('diagnosticsTable');
	if (diagnosticsTable) {
		diagnosticsTable.remove();
	}
	
	var table = document.createElement('table');
	table.id = 'diagnosticsTable';
	table.classList.add('table-responsive', 'table-bordered'); // Add CSS classes as needed

	// Create the table header row
	var headerRow = table.insertRow(0);
	var col1 = document.createElement('th');
	col1.textContent = "Diagnostic";
	headerRow.appendChild(col1);
	
	var col2 = document.createElement('th');
	col2.textContent = "Status";
	headerRow.appendChild(col2);
	
	var col3 = document.createElement('th');
	col3.textContent = "Info";
	headerRow.appendChild(col3);
	
	diagnostics.forEach(function (diagnostic, index) {
		var row = table.insertRow(index + 1);
		var cell = row.insertCell(-1);
		cell.textContent = diagnostic.name;
		
		var cell1 = row.insertCell(-1);
		cell1.textContent = diagnostic.status ? "✅" : "❌";;
		
		var cell2 = row.insertCell(-1);
		cell2.textContent = diagnostic.message;
		
	});
	
	
	tableContainer.appendChild(table);
}

function processLatencies(latencies) {
	console.log('processLatencies');
	var tableContainer = document.getElementById('latencies');
	
	var latenciesTable = document.getElementById('latenciesTable');
	if (latenciesTable) {
		latenciesTable.remove();
	}
	
	var table = document.createElement('table');
	table.id = 'latenciesTable';
	table.classList.add('table-responsive', 'table-bordered'); // Add CSS classes as needed

	// Create the table header row
	var headerRow = table.insertRow(0);
	var col1 = document.createElement('th');
	col1.textContent = "Latency";
	headerRow.appendChild(col1);
	
	var col2 = document.createElement('th');
	col2.textContent = "Quality";
	headerRow.appendChild(col2);
	
	var col3 = document.createElement('th');
	col3.textContent = "Info";
	headerRow.appendChild(col3);
	
	latencies.forEach(function (latency, index) {
		var row = table.insertRow(index + 1);
		var cell = row.insertCell(-1);
		cell.textContent = latency.name;
		
		var cell1 = row.insertCell(-1);
		cell1.textContent = latency.status ? "✅" : "❌";;
		
		var cell2 = row.insertCell(-1);
		cell2.textContent = latency.message;
		
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
	if (msg.latencies){
		processLatencies(msg.latencies);
	}
}


//******************************
//Function to get diagnostics results
//******************************
function updateDiagnostic() {
	var cmd = { command: "updateDiagnostic" };
	socket.send(JSON.stringify(cmd));
}

function getRunningNodes() {
	var cmd = { command: "getRunningNodes" };
	socket.send(JSON.stringify(cmd));
}

function doLatencyTest() {
	var cmd = { command: "doLatencyTest" };
	socket.send(JSON.stringify(cmd));
}

function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
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
	//sleep(1000);
	doLatencyTest();
}
