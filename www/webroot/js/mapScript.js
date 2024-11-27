var configSocket; // WebSocket pour les configurations
var telemetrySocket; // WebSocket pour les données de télémétrie

//Init map
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
navMap = new L.map("navMap").setView([0, 0], 23);
var polycolor = "";

//Chart.defaults.global.defaultFontFamily = 'Nunito', '-apple-system,system-ui,BlinkMacSystemFont,"Segoe UI",Roboto,"Helvetica Neue",Arial,sans-serif';
//Chart.defaults.global.defaultFontColor = '#858796'

L.control.scale().addTo(navMap);

var polynew = new L.polyline([]).addTo(navMap);
var polyline = new L.polyline([]).addTo(navMap);
var polyLatLngs = [];

var newhotline = new L.hotline([], {}).addTo(navMap);
var hotlineLayer = new L.hotline([], {}).addTo(navMap);
var hotlineLayer1 = new L.hotline([], {}).addTo(navMap);
var hotqp = [];

var curentweight = 23;
var smootf = 5;

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



var currentZoom = navMap.getZoom();



//	navMap = new L.map("navMap").setView([0, 0], 1);	
//        var shpfile = new L.Shapefile("map/TM_WORLD_BORDERS-0.3.zip", {
//                onEachFeature: function(feature, layer) {
//                        if (feature.properties) {
//                                layer.bindPopup(Object.keys(feature.properties).map(function(k) {
//                                        return k + ": " + feature.properties[k];
//                                }).join("<br />"), {
//                                        maxHeight: 200
//                                });
//                        }
//                }
//        });

//        shpfile.addTo(navMap);
//	navMap.invalidateSize();

/*
function processState(msg) {
  if (msg.telemetry.vitals[4] > oldstate) {
    oldstate = msg.telemetry.vitals[4];
    currentZoom = navMap.getZoom();
    curentweight = (23 - currentZoom) * 6 + 23;
    //positionne la carte sur la position gps
    navMap.panTo([msg.telemetry.position[1], msg.telemetry.position[0]]);

    //dessine la ligne parcourue
    latlon = [msg.telemetry.position[1], msg.telemetry.position[0]];
    qp = [msg.telemetry.position[1], msg.telemetry.position[0], msg.telemetry.depth[0]];
    currentpolyline.push(latlon);
    quakePoints.push(qp);
    polyline.remove();
    hotlineLayer.remove();
    polyline = L.polyline([currentpolyline], { weight: 2, opacity: 1, smoothFactor: smootf, color: 'red' }).addTo(navMap);
    hotlineLayer = L.hotline(quakePoints, { min: 7, max: 10, palette: { 0.0: '#ff0000', 0.2: '#ffff00', 0.4: '#00ff00', 0.6: '#00ffff', 0.8: '#0000ff', 0.9: '#8000ff', 1.0: '#000000' }, smoothFactor: smootf, weight: curentweight, outlineColor: '#000000', outlineWidth: 0 }).addTo(navMap);
    polyLatLngs = polyline.getLatLngs();

    //if (currentpolyline.length == 200){
    //        polyline1 = L.polyline(currentpolyline,{weight: 2,opacity:1, smoothFactor: smootf , color:'red'}).addTo(m);
    //        currentpolyline= [latlon];
    //        hotlineLayer1 =  L.hotline(quakePoints, {min: 7,max: 10,palette: {0.0: '#ff0000',0.2: '#ffff00',0.4: '#00ff00',0.6: '#00ffff',0.8: '#0000ff',0.9: '#8000ff', 1.0: '#000000'},smoothFactor: smootf ,weight: 20,outlineColor: '#000000', outlineWidth: 0}).addTo(m);
    //        quakePoints = [qp];
    //        }

  }
}
*/
function processState(msg) {
  // Assurez-vous que msg et msg.telemetry existent et contiennent les propriétés attendues
  if (msg && msg.telemetry && msg.telemetry.vitals && msg.telemetry.position && msg.telemetry.depth) {
      if (msg.telemetry.vitals[4] > oldstate) {
          oldstate = msg.telemetry.vitals[4];
          currentZoom = navMap.getZoom();
          curentweight = (23 - currentZoom) * 6 + 23;

          // Positionne la carte sur la position GPS
          navMap.panTo([msg.telemetry.position[1], msg.telemetry.position[0]]);

          // Dessine la ligne parcourue
          latlon = [msg.telemetry.position[1], msg.telemetry.position[0]];
          qp = [msg.telemetry.position[1], msg.telemetry.position[0], msg.telemetry.depth[0]];
          currentpolyline.push(latlon);
          quakePoints.push(qp);
          polyline.remove();
          hotlineLayer.remove();
          polyline = L.polyline([currentpolyline], { weight: 2, opacity: 1, smoothFactor: smootf, color: 'red' }).addTo(navMap);
          hotlineLayer = L.hotline(quakePoints, {
              min: 7,
              max: 10,
              palette: { 0.0: '#ff0000', 0.2: '#ffff00', 0.4: '#00ff00', 0.6: '#00ffff', 0.8: '#0000ff', 0.9: '#8000ff', 1.0: '#000000' },
              smoothFactor: smootf,
              weight: curentweight,
              outlineColor: '#000000',
              outlineWidth: 0
          }).addTo(navMap);
          polyLatLngs = polyline.getLatLngs();
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
      navMap.fitBounds(geofence.getBounds());
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

function initWebSockets() {
  // Initialisation du WebSocket pour la configuration
  configSocket = new WebSocket("ws://" + window.location.hostname + ":9004"); // Assurez-vous que le port est correct
  configSocket.onmessage = function (event) {
      var msg = JSON.parse(event.data);
      processConfigMessage(msg);
  };
  configSocket.onopen = function (event) {
      getConfig();
  };

  // Initialisation du WebSocket pour la télémétrie
  telemetrySocket = new WebSocket("ws://" + window.location.hostname + ":9002"); // Assurez-vous que le port est correct
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

