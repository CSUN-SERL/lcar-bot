#include "vehicle/uav_control.h"

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_control_test");
   gcs::UAVControl quad1{2000};

   ros::Rate loop_rate(10); //10Hz 

//    lcar_msgs::TargetGlobal target_pt;
//    target_pt.target.latitude = 47.3977255;
//    target_pt.target.longitude = 8.5456603;
//    target_pt.target.altitude = 10;

   lcar_msgs::TargetLocal target_pt; 
   target_pt.target.position.x = 0;
   target_pt.target.position.y = 0;
   target_pt.target.position.z = 2;

   target_pt.radius = 2;

   quad1.Arm(true);
   quad1.ScoutBuilding(target_pt);

   while(ros::ok())
   {
       quad1.Run();
       ros::spinOnce();
       loop_rate.sleep();
   }

   return 0;
}