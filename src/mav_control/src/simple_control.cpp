#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  ros::NodeHandle nh;

  ros::Rate loop_rate(100); //100Hz

  while(ros::ok())
  {
    SimpleControl::Land();
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void SimpleControl::Arm(bool value)
{
  ros::NodeHandle nh;

  //Create a service client for arming/disarming
  ros::ServiceClient mavros_arm_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  mavros_msgs::CommandBool arm;
  arm.request.value = value;

  //Call the service
  mavros_arm_client.call(arm);
}

void SimpleControl::Land()
{
  ros::NodeHandle nh;

  //Create a service client for landing
  ros::ServiceClient mavros_arm_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL land;

  //Call the service
  mavros_arm_client.call(land);
}

void SimpleControl::SetMode(char* mode)
{
  //TODO: Define the function
}
