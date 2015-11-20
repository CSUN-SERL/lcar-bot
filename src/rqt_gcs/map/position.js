// Initialize the Google Maps API v3
var map = new google.maps.Map(document.getElementById('map'), {
  zoom: 15,
  mapTypeId: google.maps.MapTypeId.ROADMAP
});

var marker = null;

function autoUpdate() {
  navigator.geolocation.getCurrentPosition(function(position) {
    var newPoint = new google.maps.LatLng(position.coords.latitude,
                                          position.coords.longitude);

    if (marker) {
      // Marker already created - Move it
      marker.setPosition(newPoint);
    }
    else {
      // Marker does not exist - Create it
      marker = new google.maps.Marker({
        position: newPoint,
        map: map
      });
    }

    // Center the map on the new position
    map.setCenter(newPoint);
  });

  // Call the autoUpdate() function every 5 seconds
  setTimeout(autoUpdate, 5000);
}

google.maps.event.addDomListener(window, 'load', autoUpdate);
