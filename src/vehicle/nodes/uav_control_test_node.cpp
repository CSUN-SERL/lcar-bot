#include "vehicle/uav_control.h"
#include <ros/ros.h>


gcs::UAVControl * quad;


void runCallback(const ros::TimerEvent& e)
{
    quad->Run();
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_control_test");
   ros::NodeHandle nh;
   
   
   quad = new gcs::UAVControl(2000);
   
   ros::Timer timer = nh.createTimer(ros::Duration(0.1), runCallback);
   
   ros::AsyncSpinner spinner(2);
   spinner.start();
   
   geometry_msgs::Pose target_pt1;
   target_pt1.position.x = 0.75;
   target_pt1.position.y = 0.25;
   target_pt1.position.z = 0.5;
   double yaw_angle = angles::normalize_angle_positive(angles::from_degrees(90));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle), target_pt1.orientation);
   
   geometry_msgs::Pose target_pt2;
   target_pt2.position.x = 0.75;
   target_pt2.position.y = -0.85;
   target_pt2.position.z = 0.5;
   double yaw_angle2 = angles::normalize_angle_positive(angles::from_degrees(90));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle2), target_pt2.orientation);
   
   geometry_msgs::Pose target_pt3;
   target_pt3.position.x = -0.4;
   target_pt3.position.y = -0.75;
   target_pt3.position.z = 0.5;
   double yaw_angle3 = angles::normalize_angle_positive(angles::from_degrees(180));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle3), target_pt3.orientation);
   
   geometry_msgs::Pose target_pt4;
   target_pt4.position.x = -0.35;
   target_pt4.position.y = 0.35;
   target_pt4.position.z = 0.5;
   double yaw_angle4 = angles::normalize_angle_positive(angles::from_degrees(270));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle4), target_pt4.orientation);
   
   std::vector<geometry_msgs::Pose> waypoints_list;   
   waypoints_list.push_back(target_pt1);
   waypoints_list.push_back(target_pt2);
   waypoints_list.push_back(target_pt3);
   waypoints_list.push_back(target_pt4);
//   
//   quad->SetLinearVelocity(.1,.1,.1);
//   ROS_INFO_STREAM(waypoints_list.size());
   
   
   ros::Rate(0.3).sleep();
   
   
   quad->SetMission(waypoints_list);
   
   quad->StartMission();
   
   quad->EnableOffboard();
   
   ros::waitForShutdown();
   
   return 0;
}