#include "vehicle/uav_control.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_control_test");
   gcs::UAVControl quad1{2000};


//    lcar_msgs::TargetGlobal target_pt;
//    target_pt.target.latitude = 47.3977255;
//    target_pt.target.longitude = 8.5456603;
//    target_pt.target.altitude = 10;
   
   /*lcar_msgs::TargetLocal target_pt1; 
   target_pt1.target.position.x = 2;
   target_pt1.target.position.y = 2;
   target_pt1.target.position.z = 2;
   target_pt1.radius = 2;*/
   
   geometry_msgs::Pose target_pt;
   target_pt.position.x = -2;
   target_pt.position.y = -2;
   target_pt.position.z =2;
   quaternionTFToMsg(tf::createQuaternionFromYaw(0), target_pt.orientation);
   quad1.SetTarget(target_pt);
   
   quad1.Arm(true);
   
   //quad1.ScoutBuilding(target_pt1);
   quad1.EnableOffboard();
   
   //Make a setup function
   //quad1.ScoutBuilding(target_pt);

   ros::Rate loop_rate(10); //10Hz 
  
   while(ros::ok())
   { 
       quad1.Run();
       ros::spinOnce();
       loop_rate.sleep();
   }

   return 0;
}