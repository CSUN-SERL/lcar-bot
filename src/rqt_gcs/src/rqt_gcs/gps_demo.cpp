#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rqt_gcs/simple_control.h>

#include <mavros/mavros.h>
#include <sensor_msgs/NavSatFix.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_demo");
  ros::NodeHandle nh;
  int num_uav = 2;
  int cur_uav =0;
  ros::Rate loop_rate(100); //10Hz per topic
  //std::string uav_ns = "UAV";
  ros::Publisher pub_pos_global1;
  ros::Publisher pub_pos_global2;
  float lat = 34.24203265889959;
  float lon = -118.52938055992126;
  sensor_msgs::NavSatFix position[num_uav];
  //String num = uav_ns + std::to_string(1); //UAV Namespace
  geometry_msgs::PoseStamped pos1;
  geometry_msgs::PoseStamped pos2;

  ros::Subscriber subscriber1 = nh.subscribe("UAV1/mavros/mocap/pose", QUEUE_SIZE, StateCallback1());
  ros::Subscriber subscriber2 = nh.subscribe("UAV2/mavros/mocap/pose", QUEUE_SIZE, StateCallback2());

  pub_pos_global1 = nh.advertise<sensor_msgs::NavSatFix>("UAV1/mavros/global_position/global",10);
  pub_pos_global2 = nh.advertise<sensor_msgs::NavSatFix>("UAV2/mavros/global_position/global",10);

  pos1.pose.position.y  = lat;
  pos1.pose.position.x = lon;

  while(ros::ok())
  {

    position[cur_uav].longitude += (pos1.pose.position.x * 10);
    position[cur_uav].latitude += (pos1.pose.position.x * 10);

    pub_pos_global1 = nh.advertise<sensor_msgs::NavSatFix>("UAV1/mavros/global_position/global",10);
    pub_pos_global2 = nh.advertise<sensor_msgs::NavSatFix>("UAV2/mavros/global_position/global",10);

    if(cur_uav == num_uav-1) cur_uav = 0;
    else cur_uav++;

    ros::spinOnce();
    loop_rate.sleep();
  }

}



      void StateCallback1(const geometry_msgs::PoseStamped& msg_pos) { pos1 = msg_pos.pose; }
      void StateCallback2(const geometry_msgs::PoseStamped& msg_pos) { pos2 = msg_pos.pose; }

