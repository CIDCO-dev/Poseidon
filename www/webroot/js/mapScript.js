var configSocket; // WebSocket pour les configurations
var telemetrySocket; // WebSocket pour les données de télémétrie

//Init map
var autoPan = true;
var latlon = Array(2);
var currentpolyline = Array(10);
var currentpolyline1 = Array(10);
var currentpolyline2 = Array(10);
var currentpolyline3 = Array(10);
var currentpolyline4 = Array(10);
var currentpolyline5 = Array(10);
var currentpolyline6 = Array(10);
var currentpolyline7 = Array(10);
var currentpolyline8 = Array(10);
var qp = Array(3);
var quakePoints = Array(10);
var quakePoints1 = Array(10);
var quakePoints2 = Array(10);
var quakePoints3 = Array(10);
var quakePoints4 = Array(10);
var quakePoints5 = Array(10);
var quakePoints6 = Array(10);
var quakePoints7 = Array(10);
var quakePoints8 = Array(10);
currentpolyline = currentpolyline.filter(item => item);
quakePoints = quakePoints.filter(item => item);
var oldstate = 0;
var polycolor = "";
var navMap;
var polyline;
var polyLatLngs = [];
var hotlineLayer;
var curentweight = 23;
var smootf = 5;
var currentZoom = 10;

if (typeof L !== "undefined" && typeof L.map === "function") {
  navMap = L.map("navMap").setView([0, 0], 23);

  L.control.scale().addTo(navMap);
  var legend = L.control({position: 'bottomright'});

  var polynew = new L.polyline([]).addTo(navMap);
  polyline = new L.polyline([]).addTo(navMap);
  polyLatLngs = [];

  var newhotline = new L.hotline([], {}).addTo(navMap);
  hotlineLayer = new L.hotline([], {}).addTo(navMap);
  var hotlineLayer1 = new L.hotline([], {}).addTo(navMap);
  var hotqp = [];

  curentweight = 23;
  smootf = 5;

  var shpfile = new L.Shapefile("map/ne_10m_bathymetry_L_0.zip", {
    onEachFeature: function (feature, layer) {
      if (feature.properties) {
        layer.bindPopup(Object.keys(feature.properties).map(function (k) {
          return k + ": " + feature.properties[k];
        }).join("<br />"), {
          maxHeight: 200
        });
      }
    }
  });

  shpfile.addTo(navMap);

  shpfile.once("data:loaded", function () {
    console.log("finished loaded shapefile");
  });

  legend.onAdd = function (navMap) {
              var div = L.DomUtil.create('div', 'legend');
              div.innerHTML +=
                  '<i style="background: blue"></i> Bathymetry<br>' +
                  '<i style="background: green"></i> Geofence<br>';
              return div;
          };

          legend.addTo(navMap);

  var panControl = L.control({position: 'topright'});
      panControl.onAdd = function (navMap) {
          var btn = L.DomUtil.create('button', '');
          btn.innerHTML = 'Auto Pan On';
          btn.onclick = toggleAutoPan;
          return btn;
      };
      panControl.addTo(navMap);

  if (navMap && navMap.getZoom) {
    currentZoom = navMap.getZoom();
  }
}

function processState(msg) {
  // Assurez-vous que msg et msg.telemetry existent et contiennent les propriétés attendues
  if (msg && msg.telemetry && msg.telemetry.vitals && msg.telemetry.position && msg.telemetry.depth) {
      if (msg.telemetry.vitals[4] > oldstate) {
          oldstate = msg.telemetry.vitals[4];
          currentZoom = navMap.getZoom();
          curentweight = (23 - currentZoom) * 6 + 23;

          // Positionne la carte sur la position GPS
          if (autoPan && navMap && navMap.panTo) {
               navMap.panTo([msg.telemetry.position[1], msg.telemetry.position[0]]);
          }
          // Dessine la ligne parcourue
          latlon = [msg.telemetry.position[1], msg.telemetry.position[0]];
          qp = [
            msg.telemetry.position[1],
            msg.telemetry.position[0],
            msg.telemetry.depth.length > 0 ? msg.telemetry.depth[0] : 0 // Default Value if the sonar value is not present (ex: 0)
            ];
        
          currentpolyline.push(latlon);
          quakePoints.push(qp);
          if (polyline && polyline.remove && typeof L !== "undefined") {
            polyline.remove();
            hotlineLayer.remove();
            polyline = L.polyline([currentpolyline], { weight: 2, opacity: 1, smoothFactor: smootf, color: 'red' }).addTo(navMap);
            hotlineLayer = L.hotline(quakePoints, {
              min: 7,
              max: 10,
              palette: {
                  0.0: '#ff0000',
                  0.2: '#ffff00',
                  0.4: '#00ff00',
                  0.6: '#00ffff',
                  0.8: '#0000ff',
                  0.9: '#8000ff',
                  1.0: '#000000'
              },
              smoothFactor: smootf,
              weight: curentweight,
              outlineColor: '#000000',
              outlineWidth: 0
          }).addTo(navMap);
            polyLatLngs = polyline.getLatLngs();
          }
      }
  } else {
      console.log("Invalid message or missing telemetry data");
  }
}

function processConfig(config) {
  var geofenceData;

  config.forEach(function (item) {
      switch (item.key) {
          case "geofence":
              geofenceData = item.value; // capture la définition du geofence
              drawGeofence(geofenceData); // dessine le geofence sur la carte
              break;
          // Traitez d'autres clés de configuration ici
      }
  });
}


function drawGeofence(wktString) {
  if (typeof Wkt === "undefined" || !Wkt.Wkt) {
    return;
  }
  var wkt = new Wkt.Wkt();

  try {
      wkt.read(wktString);
      var geofence = wkt.toObject({
          color: 'green',  // Couleur de la ligne
          fillColor: '#b0de5c',  // Couleur de remplissage
          fillOpacity: 0.5,  // Transparence
          weight: 2  // Épaisseur de la ligne
      });

      // Ajout à la carte
      geofence.addTo(navMap);
      if (navMap && navMap.fitBounds) {
        navMap.fitBounds(geofence.getBounds());
      }
  } catch (e) {
      console.error('Error parsing WKT', e);
  }
}



function processConfigMessage(msg) {
  if (msg.configuration) {
      processConfig(msg.configuration);
  }
}

function getConfig() {
    var cmd = { command: "getConfiguration" };
    configSocket.send(JSON.stringify(cmd));
}

function toggleAutoPan() {
        autoPan = !autoPan;
        this.innerHTML = autoPan ? 'Auto Pan On' : 'Auto Pan Off'; // Mettre à jour le texte du bouton
    }

function initWebSockets() {
  // Initialisation du WebSocket pour la configuration
  configSocket = (typeof WebSocket !== "undefined")
    ? new WebSocket("ws://" + window.location.hostname + ":9004")
    : { send: function () {} }; // Assurez-vous que le port est correct
  configSocket.onmessage = function (event) {
      var msg = JSON.parse(event.data);
      processConfigMessage(msg);
  };
  configSocket.onopen = function (event) {
      getConfig();
  };

  // Initialisation du WebSocket pour la télémétrie
  telemetrySocket = (typeof WebSocket !== "undefined")
    ? new WebSocket("ws://" + window.location.hostname + ":9002")
    : { send: function () {} }; // Assurez-vous que le port est correct
  telemetrySocket.onmessage = function (event) {
      var msg = JSON.parse(event.data);
      if (msg.telemetry) {
        processState(msg);
      }; 
  };
  telemetrySocket.onopen = function (event) {
      // Peut-être envoyer une commande pour commencer à recevoir les données de télémétrie
  };
}

// Init websocket
initWebSockets();
