#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <mavros/mavros.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_publisher");
  ros::NodeHandle nh;
  int num_uav = 99;
  int cur_uav = 0;
  int boolean = 0;
  float lat =0, lon = 0;
  ros::Rate loop_rate(100*num_uav); //10Hz per topic
  std::string uav_ns  = "UAV";
  ros::Publisher pub_pos_global[num_uav];
  sensor_msgs::NavSatFix position[num_uav];

  for(int i = 0; i < num_uav; i++){
    position[i].latitude  = lat;
    position[i].longitude = lon;

    pub_pos_global[i] = nh.advertise<sensor_msgs::NavSatFix>(uav_ns + std::to_string(i) + "/mavros/global_position/global",10);

      if( (i) % 10 == 0){
        if(i == 0){
          lat = 34.24203265889959;
          lon = -118.52938055992126;
        }else{
          lat = rand() % 170 + (-85); //-90 to 90
          lon = rand() % 355 + (-175); //-180 to 180
        }
      }else {
        lat += .001;
        lon += .001;
      }
  }

  while(ros::ok())
  {

    //if(boolean == 1){
    position[cur_uav].longitude += 0.000001;
    position[cur_uav].latitude += 0.000001;
    //boolean = boolean - 1;
  //}

    pub_pos_global[cur_uav].publish(position[cur_uav]);

    if(cur_uav == num_uav-1) cur_uav = 0;
    else cur_uav++;

    ros::spinOnce();
    loop_rate.sleep();
  }

}
