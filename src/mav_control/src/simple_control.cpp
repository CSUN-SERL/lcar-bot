#include "simple_control.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  SimpleControl quad1;
  quad1.Arm(false);

  ros::Rate loop_rate(5); //1Hz
  while(ros::ok())
  {
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
  sc_mission  = nh_simple_control.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

  //Initialize Publisher Objects
  pub_override_rc       = nh_simple_control.advertise<mavros_msgs::OverrideRCIn>("/mavros/rc/override",QUEUE_SIZE);
  pub_setpoint_position = nh_simple_control.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",QUEUE_SIZE);
  pub_setpoint_attitude = nh_simple_control.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
  pub_angular_vel       = nh_simple_control.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
}

//TODO: Pre arm/disarm checks to ensure UAV isn't already armed or airborne
void SimpleControl::Arm(bool value)
{
  //Create a message for arming/disarming
  mavros_msgs::CommandBool arm;
  arm.request.value = value;

  //Call the service
  sc_arm.call(arm);
  if(sc_arm.call(arm)){
    if(arm.response.success == 1 && value){
      ROS_INFO_STREAM("**ARMED**\n");
    }
    else if(arm.response.success == 1 && !value){
      ROS_INFO_STREAM("**DISARMED**\n");
    }
    else{
      ROS_INFO_STREAM("Failed to arm/disarm!");
    }
  }
  else{
    ROS_ERROR_STREAM("Failed to call arm service!");
  }
}

//TODO: Ensure the UAV is first armed and in guided mode. Check for success.
//TODO: Provide feedback and updates using the ROS logger
void SimpleControl::Takeoff(int altitude)
{
  //Create a message for landing
  mavros_msgs::CommandTOL takeoff;
  takeoff.request.altitude = altitude;

  //Call the service
  if(sc_takeoff.call(takeoff)){
    ROS_INFO_STREAM("Response from service: " << takeoff.response << "\n");
  }
  else{
    ROS_ERROR_STREAM("Failed to call takeoff service!");
  }
}

void SimpleControl::Land()
{
  //Create a message for landing
  mavros_msgs::CommandTOL land;

  //Call the service
  if(sc_land.call(land)){
    ROS_INFO_STREAM("Response from service: " << land.response << "\n");
  }
  else{
    ROS_ERROR_STREAM("Failed to call land service!");
  }
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
  if(sc_mode.call(new_mode)){
    ROS_INFO_STREAM("Response from service: " << new_mode.response << "\n");
  }
  else{
    ROS_ERROR_STREAM("Failed to call new_mode service!");
  }
}

//TODO: Ensure the UAV is airborne and check for service success
//TODO: Provide feedback and updates using the ROS logger
//NOTE: Deprecated in latest version of ROS
void SimpleControl::GoToWP(double lat, double lon, int alt)
{
  //Create a message for storing the the waypoint
  mavros_msgs::WaypointGOTO msg_waypoint;

  //Create the waypoint object
  mavros_msgs::Waypoint wp;
  wp.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp.is_current   = false;
  wp.autocontinue = false;
  wp.x_lat        = lat;
  wp.y_long       = lon;
  wp.z_alt        = alt;

  //Update the message with the new waypoint
  msg_waypoint.request.waypoint = wp;

  //Call the service
  if(sc_wp_goto.call(msg_waypoint)){
    ROS_INFO_STREAM("Response from service: " << msg_waypoint.response << "\n");
  }
  else{
    ROS_ERROR_STREAM("Failed to call msg_waypoint service!");
  }
}

void SimpleControl::SendMission(std::string mission_file)
{
  //Create a message for storing the the waypoint
  mavros_msgs::WaypointPush msg_mission;

  //TODO: Add ability to create waypoints from a text file.
  mavros_msgs::Waypoint wp1;
  wp1.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL;
  wp1.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp1.is_current   = false;
  wp1.autocontinue = true;
  wp1.x_lat        = -35.3632621765;
  wp1.y_long       = 149.165237427;
  wp1.z_alt        = 583.989990234;

  mavros_msgs::Waypoint wp2;
  wp2.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp2.command      = mavros_msgs::CommandCode::NAV_TAKEOFF;
  wp2.is_current   = false;
  wp2.autocontinue = true;
  wp2.x_lat        = -35.3628807068;
  wp2.y_long       = 149.165222168;
  wp2.z_alt        = 20;

  mavros_msgs::Waypoint wp3;
  wp3.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
  wp3.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
  wp3.is_current   = false;
  wp3.autocontinue = true;
  wp3.x_lat        = -35.3646507263;
  wp3.y_long       = 149.163497925;
  wp3.z_alt        = 0;

  //Update the message with the new waypoint
  //NOTE: waypoints is a Vector object
  msg_mission.request.waypoints.push_back(wp1);
  msg_mission.request.waypoints.push_back(wp2);
  msg_mission.request.waypoints.push_back(wp3);

  //Call the service
  if(sc_mission.call(msg_mission)){
    ROS_INFO_STREAM("Response from service: " << msg_mission.response << "\n");
  }
  else{
    ROS_ERROR_STREAM("Failed to call msg_mission service!");
  }
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

void SimpleControl::SetAngularVelocity(int roll_vel, int pitch_vel, int yaw_vel)
{
  //Create the message object
  geometry_msgs::TwistStamped msg_angular_vel;

  //Update the message with the new angular velocity
  geometry_msgs::Twist velocity;
  velocity.angular.x = roll_vel;
  velocity.angular.y = pitch_vel;
  velocity.angular.z = yaw_vel;
  msg_angular_vel.twist = velocity;

  //Publish the message
  pub_angular_vel.publish(msg_angular_vel);
}
