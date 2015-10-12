#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  ros::NodeHandle nh;

  ros::Rate loop_rate(1); //1Hz
  while(ros::ok())
  {
    SimpleControl::OverrideRC(3, 2000, &nh);
    ros::spinOnce();
    loop_rate.sleep();
  }

}

SimpleControl::SimpleControl(void)
{
  //Class constructor
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
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

void SimpleControl::OverrideRC(int channel, int value, ros::NodeHandle* nh)
{
  //Create the publisher and message objects
  ros::Publisher override_rc_pub = nh->advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",QUEUE_SIZE);
  mavros_msgs::OverrideRCIn override_msg;

  // Update the message with the new RC value
  override_msg.channels[channel-1] = value;

  //Publish the message
  override_rc_pub.publish(override_msg);
}

void SimpleControl::SetLocalPosition(int x, int y, int z, ros::NodeHandle* nh)
{
  //Create the publisher and message objects
  ros::Publisher setpoint_position_pub  = nh->advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",QUEUE_SIZE);
  geometry_msgs::PoseStamped position_stamped;

  //Update the message for the new position
  geometry_msgs::Pose point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  position_stamped.pose = point;

  //Publish the message
  setpoint_position_pub.publish(position_stamped);
}
