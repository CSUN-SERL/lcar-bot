

	var map = new GMap2(document.getElementById("googleMap"));
	      map.addControl(new GMapTypeControl());
	      map.setCenter(new GLatLng(0,0),2);
	      var dirn = new GDirections();
	      var step = 5; // metres
	      var tick = 100; // milliseconds
	      var poly;
	      var eol;
	      var car = new GIcon();
	          car.image="uav.png"
	          car.iconSize=new GSize(32,18);
	          car.iconAnchor=new GPoint(16,9);
	      var marker;
	      var k=0;
	      var stepnum=0;
	      var speed = "";

	      function animate(d) {
	        if (d>eol) {
	          document.getElementById("step").innerHTML = "<b>Trip completed<\/b>";
	          document.getElementById("distance").innerHTML =  "Miles: "+(d/1609.344).toFixed(2);
	          return;
	        }
	        var p = poly.GetPointAtDistance(d);
	        if (k++>=180/step) {
	          map.panTo(p);
	          k=0;
	        }
	        marker.setPoint(p);
	        document.getElementById("distance").innerHTML =  "Miles: "+(d/1609.344).toFixed(2)+speed;
	        if (stepnum+1 < dirn.getRoute(0).getNumSteps()) {
	          if (dirn.getRoute(0).getStep(stepnum).getPolylineIndex() < poly.GetIndexAtDistance(d)) {
	            stepnum++;
	            var steptext = dirn.getRoute(0).getStep(stepnum).getDescriptionHtml();
	            document.getElementById("step").innerHTML = "<b>Next:<\/b> "+steptext;
	            var stepdist = dirn.getRoute(0).getStep(stepnum-1).getDistance().meters;
	            var steptime = dirn.getRoute(0).getStep(stepnum-1).getDuration().seconds;
	            var stepspeed = ((stepdist/steptime) * 2.24).toFixed(0);
	            step = stepspeed/2.5;
	            speed = "<br>Current speed: " + stepspeed +" mph";
	          }
	        } else {
	          if (dirn.getRoute(0).getStep(stepnum).getPolylineIndex() < poly.GetIndexAtDistance(d)) {
	            document.getElementById("step").innerHTML = "<b>Next: Arrive at your destination<\/b>";
	          }
	        }
	        setTimeout("animate("+(d+step)+")", tick);
	      }

	      GEvent.addListener(dirn,"load", function() {
	        document.getElementById("controls").style.display="none";
	        poly=dirn.getPolyline();
	        eol=poly.Distance();
	        map.setCenter(poly.getVertex(0),17);
	        map.addOverlay(new GMarker(poly.getVertex(0),G_START_ICON));
	        map.addOverlay(new GMarker(poly.getVertex(poly.getVertexCount()-1),G_END_ICON));
	        marker = new GMarker(poly.getVertex(0),{icon:car});
	        map.addOverlay(marker);
	        var steptext = dirn.getRoute(0).getStep(stepnum).getDescriptionHtml();
	        document.getElementById("step").innerHTML = steptext;
	        setTimeout("animate(0)",2000);  // Allow time for the initial map display
	      });

	      GEvent.addListener(dirn,"error", function() {
	        alert("Location(s) not recognised. Code: "+dirn.getStatus().code);
	      });

	      function start() {
	        var startpoint = document.getElementById("startpoint").value;
	        var endpoint = document.getElementById("endpoint").value;
	        dirn.loadFromWaypoints([startpoint,endpoint],{getPolyline:true,getSteps:true});
	      }

	  
