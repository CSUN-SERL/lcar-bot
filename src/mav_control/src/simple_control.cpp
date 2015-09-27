#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  ros::NodeHandle nh;

  ros::ServiceClient mavros_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool arm;
  arm.request.value = true;
  mavros_arm_client.call(arm);

  ros::Rate loop_rate(100.0);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}
