#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  ros::NodeHandle nh;

  ros::Rate loop_rate(0.5); //100Hz

  while(ros::ok())
  {
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
  ros::ServiceClient mavros_land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL land;

  //Call the service
  mavros_land_client.call(land);
}

void SimpleControl::SetMode(std::string mode)
{
  char new_custom_mode;

  // Set new_custom_mode for the desired flight mode
  if(mode.compare("Stabilize")       == 0)  new_custom_mode = '0';
  else if(mode.compare("Alt Hold")   == 0)  new_custom_mode = '2';
  else if(mode.compare("Auto")       == 0)  new_custom_mode = '3';
  else if(mode.compare("Guided")     == 0)  new_custom_mode = '4';
  else if(mode.compare("Loiter")     == 0)  new_custom_mode = '5';
  else if(mode.compare("RTL")        == 0)  new_custom_mode = '6';
  else if(mode.compare("Circle")     == 0)  new_custom_mode = '7';
  else                                      new_custom_mode = '0';

  ros::NodeHandle nh;

  //Create a service client for changing flight mode
  ros::ServiceClient mavros_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  mavros_msgs::SetMode new_mode;
  new_mode.request.base_mode = 0;
  new_mode.request.custom_mode = new_custom_mode; //custom_mode expects a char*

  //Call the service
  mavros_mode_client.call(new_mode);
}
