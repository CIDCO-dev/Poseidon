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
	if (socket && socket.send) {
		socket.send(JSON.stringify(cmd));
	}
}

function getRunningNodes() {
	var cmd = { command: "getRunningNodes" };
	if (socket && socket.send) {
		socket.send(JSON.stringify(cmd));
	}
}


//******************************
// Main
//******************************
socket = (typeof WebSocket !== "undefined")
	? new WebSocket("ws://" + window.location.hostname + ":9099")
	: { send: function () {} };

if (socket && socket.onmessage !== undefined) {
	socket.onmessage = function (event) {
		var msg = JSON.parse(event.data);
		processMessage(msg);
	}

	socket.onopen = function (event) {
		getRunningNodes();
		updateDiagnostic();
	}
}
	
	var diagnosticsButton = document.getElementById("diagnosticsButton");
	if (diagnosticsButton) {
		diagnosticsButton.addEventListener("click",function(){
			getRunningNodes();
			updateDiagnostic();
		});
	}
	
	
