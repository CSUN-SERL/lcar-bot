#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  SimpleControl quad1;

  /*quad1.SetMode("Guided");
  quad1.Arm(true);
  quad1.Takeoff(5);

  ros::Rate delay(0.25);
  delay.sleep();
  quad1.GoToWP(-35.362881, 149.165222, 10);*/

  ros::Rate loop_rate(5); //1Hz

  while(ros::ok())
  {
    quad1.SetAttitude(30,50,20);
    ros::spinOnce();
    loop_rate.sleep();
  }

}

SimpleControl::SimpleControl(void)  //Class constructor
{
  //Initialize Service Clients
  sc_arm      = nh_simple_control.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  sc_takeoff  = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  sc_land     = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  sc_mode     = nh_simple_control.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  sc_wp_goto  = nh_simple_control.serviceClient<mavros_msgs::WaypointGOTO>("/mavros/mission/goto");

  //Initialize Publisher Objects
  pub_override_rc       = nh_simple_control.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",QUEUE_SIZE);
  pub_setpoint_position = nh_simple_control.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",QUEUE_SIZE);
  pub_setpoint_attitude = nh_simple_control.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
}

//TODO: Pre arm/disarm checks to ensure UAV isn't already armed or airborne
//TODO: Provide feedback and updates using the ROS logger
void SimpleControl::Arm(bool value)
{
  //Create a message for arming/disarming
  mavros_msgs::CommandBool arm;
  arm.request.value = value;

  //Call the service
  sc_arm.call(arm);
}

//TODO: Ensure the UAV is first armed and in guided mode. Check for success.
//TODO: Provide feedback and updates using the ROS logger
void SimpleControl::Takeoff(int altitude)
{
  //Create a message for landing
  mavros_msgs::CommandTOL takeoff;
  takeoff.request.altitude = altitude;

  //Call the service
  sc_takeoff.call(takeoff);
}

void SimpleControl::Land()
{
  //Create a message for landing
  mavros_msgs::CommandTOL land;

  //Call the service
  sc_land.call(land);
}

//TODO: Check for service success
//TODO: Provide feedback and updates using the ROS logger
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

  //Create a message for changing flight mode
  mavros_msgs::SetMode new_mode;
  new_mode.request.base_mode = 0;
  new_mode.request.custom_mode = new_custom_mode; //custom_mode expects a char*

  //Call the service
  sc_mode.call(new_mode);
}

//TODO: Ensure the UAV is airborne and check for service success
//TODO: Provide feedback and updates using the ROS logger
void SimpleControl::GoToWP(double lat, double lon, int alt)
{
  //Create a message for storing the the waypoint
  mavros_msgs::WaypointGOTO msg_waypoint;

  //Create the waypoint object
  //TODO: Use the enum macros instead of magic numbers
  mavros_msgs::Waypoint wp;
  wp.frame        = 0;    //GLOBAL_FRAME
  wp.command      = 16;   //WP_NAV
  wp.is_current   = false;
  wp.autocontinue = false;
  wp.x_lat        = lat;
  wp.y_long       = lon;
  wp.z_alt        = alt;

  //Update the message with the new waypoint
  msg_waypoint.request.waypoint = wp;

  //Call the service
  sc_wp_goto.call(msg_waypoint);
}

void SimpleControl::OverrideRC(int channel, int value)
{
  //Create the message object
  mavros_msgs::OverrideRCIn override_msg;

  // Update the message with the new RC value
  override_msg.channels[channel-1] = value;

  //Publish the message
  pub_override_rc.publish(override_msg);
}

void SimpleControl::SetLocalPosition(int x, int y, int z)
{
  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  geometry_msgs::Pose point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  position_stamped.pose = point;

  //Publish the message
  pub_setpoint_position.publish(position_stamped);
}

void SimpleControl::SetAttitude(int roll, int pitch, int yaw)
{
  //Create the message object
  geometry_msgs::PoseStamped msg_attitude;

  //Create an object with the new attitude
  //NOTE: Arbitrary values until the function is tested
  geometry_msgs::Pose uav_attitude;
  uav_attitude.position.x     = roll;
  uav_attitude.position.y     = pitch;
  uav_attitude.position.z     = yaw;
  uav_attitude.orientation.x  = roll;
  uav_attitude.orientation.y  = pitch;
  uav_attitude.orientation.z  = yaw;
  uav_attitude.orientation.w  = 0;

  //Update the message with the new attitude
  msg_attitude.pose = uav_attitude;

  //Publish the message
  pub_setpoint_attitude.publish(msg_attitude);
}
