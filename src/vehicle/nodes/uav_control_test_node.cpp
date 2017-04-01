#include "vehicle/uav_control.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_control_test");
   gcs::UAVControl quad1(2000);
   
   geometry_msgs::Pose target_pt1;
   target_pt1.position.x = -5;
   target_pt1.position.y = -5;
   target_pt1.position.z = 5;
   double yaw_angle = angles::normalize_angle_positive(angles::from_degrees(0));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle), target_pt1.orientation);
   
   geometry_msgs::Pose target_pt2;
   target_pt2.position.x = -4;
   target_pt2.position.y = 4;
   target_pt2.position.z = 4;
   double yaw_angle2 = angles::normalize_angle_positive(angles::from_degrees(90));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle2), target_pt2.orientation);
   
   geometry_msgs::Pose target_pt3;
   target_pt3.position.x = 3;
   target_pt3.position.y = 3;
   target_pt3.position.z = 3;
   double yaw_angle3 = angles::normalize_angle_positive(angles::from_degrees(180));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle3), target_pt3.orientation);
   
   geometry_msgs::Pose target_pt4;
   target_pt4.position.x = 2;
   target_pt4.position.y = -2;
   target_pt4.position.z = 2;
   double yaw_angle4 = angles::normalize_angle_positive(angles::from_degrees(270));
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle4), target_pt4.orientation);
   
   
   std::vector<geometry_msgs::Pose> waypoints_list;
   
   waypoints_list.push_back(target_pt1);
   waypoints_list.push_back(target_pt2);
   waypoints_list.push_back(target_pt3);
   waypoints_list.push_back(target_pt4);
   
   quad1.SetLinearVelocity(.1,.1,.1);
   ROS_INFO_STREAM(waypoints_list.size());
   
   quad1.SetMission(waypoints_list);
   
   quad1.StartMission();
   
   quad1.EnableOffboard();

   ros::Rate loop_rate(10); //10Hz 

   while(ros::ok())
   { 
       quad1.Run();
       ros::spinOnce();
       loop_rate.sleep();

   }
   
   return 0;
}