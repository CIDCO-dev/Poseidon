
var socket;

function processConfig(config) {
  var form = document.getElementById('formContent');

  form.innerHTML = '';
  var newHTML = '';
  var endNewHTML = '';

  config.forEach(function (item, index) {
    if (item.key == "loggingMode") {
      endNewHTML += '<div class="form-group row align-items-center"><label class="col-auto col-form-label" for="' + item.key + '">' + item.key + " : " + '</label><div class="col"><select id="' + item.key + '" name="logginModeSelector" class="dropdownright configurationField"></div>';

      var mode = Number(item.value);
      switch (mode) {
        case 1:
          endNewHTML += '<option value="1" selected>always</option><option value="2">manual</option><option value="3">speed based</option></select></div></div>';
          break;
        case 2:
          endNewHTML += '<option value="1">always</option><option value="2" selected>manual</option><option value="3">speed based</option></select></div></div>';
          break;
        case 3:
          endNewHTML += '<option value="1">always</option><option value="2">manual</option><option value="3" selected>speed based</option></select></div></div>';
          break;
        default:
          alert("Erreur ! Logging mode read from configuration file is :" + mode);
          endNewHTML += '<option value="1" selected>always ON</option><option value="2">manual</option><option value="3">speed based</option></select></div></div>';
          break;
      }
    }
    else {
      newHTML += '<div class="form-group row align-items-center"> <label class="col-auto col-form-label" for="' + item.key + '">' + item.key + " : " + '</label> <div class="col"> <input id="' + item.key + '" class="form-control configurationField" type="text" value="' + item.value + '"/> </div></div>';
    }

  });
  newHTML += endNewHTML;
  form.innerHTML = newHTML;
}

function processMessage(msg) {
  if (msg.configuration) {
    processConfig(msg.configuration);
  }
}

//******************************
//Function to save configuration
//******************************
function saveConfig() {
  var cmd = { command: "saveConfiguration", configuration: [] };

  var elements = document.getElementsByClassName("configurationField");

  var i;
  for (i = 0; i < elements.length; i++) {
    var setting = { key: elements[i].id, value: elements[i].value };
    cmd.configuration.push(setting);
  }

  console.log(cmd);
  socket.send(JSON.stringify(cmd));

}

//******************************
//Function to get configuration
//******************************
function getConfig() {
  var cmd = { command: "getConfiguration" };
  socket.send(JSON.stringify(cmd));
}

//******************************
// Main
//******************************

socket = new WebSocket("ws://" + window.location.hostname + ":9004");

socket.onmessage = function (event) {
  var msg = JSON.parse(event.data);
  processMessage(msg);
}

socket.onopen = function (event) {
  getConfig();
}