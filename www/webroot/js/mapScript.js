var socket;

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

var shpfile = new L.Shapefile("map/TM_WORLD_BORDERS-0.3.zip", {
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



// Init websocket

socket = new WebSocket("ws://" + window.location.hostname + ":9002");

socket.onmessage = function (event) {
  //console.log(event.data);
  var msg = JSON.parse(event.data);

  // Traitement des messages contenant des informations de télémétrie
  if (msg.telemetry) {
      processState(msg);
  } 
  // Traitement des messages contenant des informations d'enregistrement
  else if (msg.recordingInfo && msg.loggingMode) {
      console.log("Not used message");
  } else {
      console.log("Invalid message or missing telemetry data");
  }
}