var longitude = 2;
var latitude  = 2;
var num =2;
var listeners =[];
var total = 99;
var sections = 10;
var done = false;
var ChoosenOne;

// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros({ //creating ros object
  url : 'ws://localhost:9090'
});

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
});




$(document).ready(function(){
showPosition();
});




var x =0;
var map;
var marker =[];
var newPoint ;
var la ;
var squadCreation = 0;
var team = ["Alpha", "Beta", "Shield", "Raid", "Airborne", "Batman", "Daemon", "Nemesis", "Viper", "Cyclone" ];

var image =[];
var list = 0;
var limit =0;



function showPosition() {
//  console.log('latitude ' + longitude);
// console.log('longitude ' + latitude);


latlon = new google.maps.LatLng(longitude, latitude)
mapholder = document.getElementById('mapholder')
//mapholder.style.height = '100%';
//mapholder.style.width = '100%';


var myOptions = {
  center:latlon,zoom:2,
  mapTypeId:google.maps.MapTypeId.SATELLITE,
  mapTypeControl:true,
  navigationControlOptions:{style:google.maps.NavigationControlStyle.SMALL}
}

map = new google.maps.Map(mapholder, myOptions);
//setTimeout("update()", 4000)
update();

}

function update() {

if(squadCreation >= 0){
  //console.log("num: " + num + "squadCreation: " + squadCreation);

listener = new ROSLIB.Topic({
  ros : ros, 	//ros object connection
  name : '/UAV' + num + '/mavros/global_position/global',
  messageType : 'sensor_msgs/NavSatFix'
});

listener.subscribe(function(message) {

  if(message.latitude != 0 && message.longitude != 0){ //if not 0
    longitude = message.longitude;
    latitude = message.latitude;

    listener.unsubscribe();
  }

});
//console.log("input name uav length = " + $('input[name=uav]').length);

if(longitude != 0 && latitude != 0 && $('input[name=uav]').length <= num )  {
    //create select list buttons

    if(squadCreation == 10){
      squadCreation = 0;
    }
  //console.log("squadCreation: " + squadCreation);
    if(squadCreation == 0 && limit < 10 && done == false) {

      $('<select />', {
        id: team[list],
        name : team[list],
        value: team[list],
      }).appendTo('#radB');
      $('<input />', {
                 name: 'uav',
                 id: 'uav' + (num),
                 type: "radio",
                 value: list  ,
             }).appendTo('#radB');

            $('<label />', {
             for: 'uav' + (num),
             text: 'uav ' + (num) + ' ',
         }).appendTo('#'+team[list]);
      list++;
      limit++;
    }


if( done == false){
  if(list-1 == 0 ){
    //console.log("squadcreationg at option list-1: " + squadCreation);
    $('#Alpha').append($('<option>', {
     value:team[list-1] + (squadCreation  ),
     text :  team[list-1] + " " +  (squadCreation )
 }));

}
if(list-1 == 1){
$('#Beta').append($('<option>', {
value:team[list-1] + (squadCreation ),
text : team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 2){
$('#Shield').append($('<option>', {
value:team[list-1] + (squadCreation ),
text :team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 3){
$('#Raid').append($('<option>', {
value:team[list-1] + (squadCreation ),
text : team[list-1] + " " + (squadCreation )
}));
}

if(list-1 == 4){
$('#Airborne').append($('<option>', {
value:team[list-1] + (squadCreation ),
text : team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 5){
$('#Batman').append($('<option>', {
value:team[list-1] + (squadCreation ),
text :team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 6){
$('#Daemon').append($('<option>', {
value:team[list-1] + (squadCreation ),
text :team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 7){
$('#Nemesis').append($('<option>', {
value:team[list-1] + (squadCreation ),
text :team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 8){
$('#Viper').append($('<option>', {
value:team[list-1] + (squadCreation ),
text : team[list-1] + " " + (squadCreation )
}));
}
if(list-1 == 9){
$('#Cyclone').append($('<option>', {
value:team[list-1] + (squadCreation),
text :team[list-1] + " " + (squadCreation )
}));
}
}

//console.log(list + " this");

}

//console.log("lat: " + latitude);
//console.log("lon: " + longitude);
la = new google.maps.LatLng(latitude,
  longitude);
//console.log("num " + num);
  if (marker[num]) {
    // Marker already created - Move it
    marker[num].setPosition(la);
  //  console.log("OLD" + num);
  }
  else {
    //add new image

      image[num] = {
        url: 'images/uavIcon' + (list -1 ) + '.png',
        scaledSize: new google.maps.Size(60,60)
      };

    // Marker does not exist - Create it
      marker[num] = new google.maps.Marker({
      position: la,
      map: map,
      title: team[list -1] + " " + (squadCreation),
      icon: image[num]
    });
    //console.log("NEW" + num);
  }

  // Center the map on the new position depending on the UAV
  var checked = $('input[name=uav]:checked').val();
  if(checked != undefined && checked != "free" && checked.length < 2){
  checked = parseInt((checked + '').charAt(0));
}
//  console.log((checked )  + " checked");

  var index = team[checked ];

  var selected = $('#'+index).val();
  //console.log(index + " " + (checked) + selected);

  if(selected != undefined){
    var datNum = selected.slice(-1); //get second word, the number
}
  //console.log(datNum + " the numba!");

  if(checked  == 0){
//  console.log("varNum +2 = " + (datNum + 2));

        theChoosenOne = (parseInt(datNum) + 2  ).toString();
  } else{
    if(parseInt(datNum) < 8) {
      theChoosenOne =    (parseInt(checked) ).toString() + (parseInt(datNum) +2).toString();

      console.log("UAV: " + theChoosenOne);
    } else {
      if(parseInt(datNum == 8)){

        theChoosenOne =  parseInt(checked + 1).toString() + 0 ;
        console.log("UAV else: " + theChoosenOne);
      }else{

        theChoosenOne =  parseInt(checked + 1).toString() + 1 ;
        console.log("UAV else: " + theChoosenOne);
      }
    }
   //console.log("UAV: " + (checked)+datNum);
  //  console.log("checked != 0");


}

//  console.log(theChoosenOne + " ChoosenOne");
  if(checked != 'free' && checked != undefined ) { //if its not free roam, then center map to an UAV
    map.setCenter(marker[theChoosenOne].getPosition());
  }
  else{
  //nothing
  }

  //outmessage.innerHTML = num + "  " + la ;
  if(index != undefined){
    uavName.innerHTML = index + " " + datNum;
 }

  num++
}//squad end if
  if(squadCreation < 10){
    squadCreation++;
}
else {
  done = true;
}
//console.log(num + "TOTAL");
  if(num > total){
    num = 0;
  }


  // Call the autoUpdate() function every k seconds
  setTimeout(update,70);
  //optimal 100
}



function showError(error) {
  switch(error.code) {
    case error.PERMISSION_DENIED:
    x.innerHTML = "User denied the request for Geolocation."
    break;
    case error.POSITION_UNAVAILABLE:
    x.innerHTML = "Location information is unavailable."
    break;
    case error.TIMEOUT:
    x.innerHTML = "The request to get user location timed out."
    break;
    case error.UNKNOWN_ERROR:
    x.innerHTML = "An unknown error occurred."
    break;
  }
}

function nameThem( x, squadCreation ) {

  //Alpha", "Beta", "Shield", "Raid", "Airborne", "Batman", "Daemon", "Nemesis", "Viper", "Cyclone"

}
