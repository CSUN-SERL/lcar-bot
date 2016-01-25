#include <rqt_gcs/simple_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  SimpleControl quad1;

  //quad1.ScoutBuilding(7,4,1);
  boost::thread_group tg;
  ros::Rate loop_rate(10); //10Hz

  while(ros::ok())
  {
    //Let the quad do it's current mission
    //quad1.Run();

    ros::spinOnce();
    loop_rate.sleep();
  }

}

SimpleControl::SimpleControl(void)  //Class constructor
{
  //Initialize Service Clients
  sc_arm      = nh_simple_control.serviceClient<mavros_msgs::CommandBool>(uav_ns + "/mavros/cmd/arming");
  sc_takeoff  = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(uav_ns + "/mavros/cmd/takeoff");
  sc_land     = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(uav_ns + "/mavros/cmd/land");
  sc_mode     = nh_simple_control.serviceClient<mavros_msgs::SetMode>(uav_ns + "/mavros/set_mode");
  sc_mission  = nh_simple_control.serviceClient<mavros_msgs::WaypointPush>(uav_ns + "/mavros/mission/push");

  //Initialize Publisher Objects
  pub_override_rc       = nh_simple_control.advertise<mavros_msgs::OverrideRCIn>(uav_ns + "/mavros/rc/override",QUEUE_SIZE);
  pub_setpoint_position = nh_simple_control.advertise<geometry_msgs::PoseStamped>(uav_ns + "/mavros/setpoint_position/local",QUEUE_SIZE);
  pub_setpoint_attitude = nh_simple_control.advertise<geometry_msgs::PoseStamped>(uav_ns + "/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
  pub_angular_vel       = nh_simple_control.advertise<geometry_msgs::TwistStamped>(uav_ns + "/mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
  pub_linear_vel        = nh_simple_control.advertise<geometry_msgs::TwistStamped>(uav_ns + "/mavros/setpoint_velocity/cmd_vel",QUEUE_SIZE);
  pub_setpoint_accel    = nh_simple_control.advertise<geometry_msgs::Vector3Stamped>(uav_ns + "/mavros/setpoint_accel/accel",QUEUE_SIZE);

  //Initialze Subscribers
  sub_state       = nh_simple_control.subscribe(uav_ns + "/mavros/state", 1, &SimpleControl::StateCallback, this);
  sub_battery     = nh_simple_control.subscribe(uav_ns + "/mavros/battery", 1, &SimpleControl::BatteryCallback, this);
  sub_imu         = nh_simple_control.subscribe(uav_ns + "/mavros/sensor_msgs/Imu", 1, &SimpleControl::ImuCallback, this);
  sub_altitude    = nh_simple_control.subscribe(uav_ns + "/mavros/global_position/rel_alt", 1, &SimpleControl::RelAltitudeCallback, this);
  sub_heading     = nh_simple_control.subscribe(uav_ns + "/mavros/global_position/compass_hdg", 1, &SimpleControl::HeadingCallback, this);
  sub_vel         = nh_simple_control.subscribe(uav_ns + "/mavros/local_position/velocity", 1, &SimpleControl::VelocityCallback, this);
  sub_pos_global  = nh_simple_control.subscribe(uav_ns + "/mavros/global_position/global", 1, &SimpleControl::NavSatFixCallback, this);
  sub_pos_local   = nh_simple_control.subscribe(uav_ns + "/mavros/local_position/pose", 1, &SimpleControl::LocalPosCallback, this);

  //Set Home position
  pos_home.x = pos_home.y = pos_home.z = 0;
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
}

void SimpleControl::Arm(bool value, int uav_num)
{
  if(state[uav_num].armed != value){ //Only change to new state if it's different
    //Create a message for arming/disarming
    mavros_msgs::CommandBool arm;
    arm.request.value = value;

    //Call the service
    if(sc_arm.call(arm)){
      if(arm.response.success == 1){

        bool timeout = false;
        int count = 0;
        ros::Rate check_frequency(CHECK_FREQUENCY);

        //Wait for the FCU to arm
        while(!state[uav_num].armed && !timeout){
          check_frequency.sleep();
          ros::spinOnce();
          count++;
          if(count >= TIMEOUT) timeout = true;
        }

        //Print proper message to console
        if(timeout){
          if(value) ROS_WARN_STREAM("Arm operation timed out.");
          else ROS_WARN_STREAM("Disarm operation timed out.");
        }
        else{
          if(state[uav_num].armed) ROS_INFO_STREAM("**ARMED**");
          else ROS_INFO_STREAM("**DISARMED**");
        }
      }
      else{
        if(value) ROS_ERROR_STREAM("Failed to Arm!");
        else ROS_ERROR_STREAM("Failed to Disarm!");
      }
    }
    else{
      ROS_ERROR_STREAM("Failed to call arm service!");
    }
  }
}

void SimpleControl::Takeoff(int altitude, int uav_num)
{
  //Ensure the UAV is in Guided mode and armed
  bool armed = (bool)state[uav_num].armed;
  std::string mode = state[uav_num].mode;

  if(mode.compare("GUIDED") != 0) this->SetMode("Guided", uav_num);
  if(!armed) this->Arm(true, uav_num);

  //Create a message for landing
  mavros_msgs::CommandTOL takeoff;
  takeoff.request.altitude = altitude;
;
  //Call the service
  if(sc_takeoff.call(takeoff)){
    if(takeoff.response.success == 1) ROS_INFO_STREAM("Takeoff Initiated.");
    else ROS_ERROR_STREAM("Failed to initiate takeoff.");
  }
  else{
    ROS_ERROR_STREAM("Failed to call takeoff service!");
  }
}

void SimpleControl::Land(int uav_num)
{
  //Create a message for landing
  mavros_msgs::CommandTOL land;

  //Call the service
  if(sc_land.call(land)){
    if(land.response.success == 1) ROS_INFO_STREAM("Land Initiated.");
    else ROS_ERROR_STREAM("Failed to initiate land.");
  }
  else{
    ROS_ERROR_STREAM("Failed to call land service!");
  }
}

void SimpleControl::SetMode(std::string mode, int uav_num)
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
    if(new_mode.response.success == 1) ROS_INFO_STREAM("Mode changed to " << mode << ".");
    else ROS_ERROR_STREAM("Failed to change flight mode to " << mode << ".");
  }
  else{
    ROS_ERROR_STREAM("Failed to call new_mode service!");
  }
}

std::string SimpleControl::GetLocation(int uav_num)
{
  float lat = pos_global.latitude;
  float lon = pos_global.longitude;

  return std::to_string(lat) + "," + std::to_string(lon);
}

void SimpleControl::ScoutBuilding(int x, int y, int z, int uav_num)
{
  //Update the target location
  pos_target.x = x;
  pos_target.y = y;
  pos_target.z = z;

  //Prepare the vehicle for traveling to the waypoint
  //this->Arm(true);
  //this->SetMode("Guided");

  pos_previous = pos_local;
  goal = TRAVEL;
  ROS_INFO_STREAM("Traveling to target location.");
}

void SimpleControl::OverrideRC(int channel, int value, int uav_num)
{
  //Create the message object
  mavros_msgs::OverrideRCIn override_msg;

  // Update the message with the new RC value
  override_msg.channels[channel-1] = value;

  //Publish the message
  pub_override_rc.publish(override_msg);
}

void SimpleControl::SetLocalPosition(int x, int y, int z, int uav_num)
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

void SimpleControl::SetLocalPosition(geometry_msgs::Point new_point, int uav_num)
{
  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  position_stamped.pose.position = new_point;

  //Publish the message
  pub_setpoint_position.publish(position_stamped);
}

void SimpleControl::SetAttitude(float roll, float pitch, float yaw, int uav_num)
{
  //Create the message to be published
  geometry_msgs::PoseStamped msg_pose;

  //Construct a Quaternion from Fixed angles and update pose
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  quaternionTFToMsg(q, msg_pose.pose.orientation);

  //Publish the message
  pub_setpoint_attitude.publish(msg_pose);
}

void SimpleControl::SetAngularVelocity(int roll_vel, int pitch_vel, int yaw_vel, int uav_num)
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

void SimpleControl::SetLinearVelocity(float x, float y, float z, int uav_num)
{
  geometry_msgs::TwistStamped msg_linear_vel;

  msg_linear_vel.twist.linear.x = x;
  msg_linear_vel.twist.linear.y = y;
  msg_linear_vel.twist.linear.z = z;

  pub_linear_vel.publish(msg_linear_vel);
}

void SimpleControl::SetAcceleration(float x, float y, float z, int uav_num)
{
  //Create the message object
  geometry_msgs::Vector3Stamped msg_accel;

  //Update the message with the new acceleration
  msg_accel.vector.x = x;
  msg_accel.vector.y = y;
  msg_accel.vector.z = z;

  //Publish the message
  pub_setpoint_accel.publish(msg_accel);
}

//TODO: Fix Roll, Pitch, Yaw, and Ground Speed values
FlightState SimpleControl::UpdateFlightState(int uav_num)
{
    struct FlightState flight_state;

    flight_state.roll = imu.orientation.x; //Update Roll value
    flight_state.pitch = imu.orientation.y; //Update Pitch Value
    flight_state.yaw = imu.orientation.z; //Update Yaw Value
    flight_state.heading = heading_deg; //Update heading [degrees]
    flight_state.altitude = altitude_rel; //Update Altitude [m]
    flight_state.ground_speed = velocity.twist.linear.x; //Global Velocity X [m/s]
    flight_state.vertical_speed = velocity.twist.linear.z; //Global Velocity vertical [m/s]

    return flight_state;
}

int SimpleControl::ComparePosition(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
    int result;

    if( abs(point2.x - point1.x) <= THRESHOLD_XY &&
        abs(point2.y - point1.y) <= THRESHOLD_XY &&
        abs(point2.z - point1.z) <= THRESHOLD_Z){
      result = 0;
    }
    else result = 1;

    return result;
}

int SimpleControl::CalculateDistance(geometry_msgs::Point point1, geometry_msgs::Point point2)
{
  float dist_x = point2.x - point1.x;
  float dist_y = point2.y - point1.y;
  float dist_z = point2.z - point1.z;
  return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
}

float SimpleControl::GetMissionProgress()
{
  float progress = 0;

  if(goal == TRAVEL){
    float distance_remaining  = CalculateDistance(pos_target,pos_local);
    float distance_total      = CalculateDistance(pos_target,pos_home);
    float distance_completion = distance_remaining/distance_total;
    progress =  TRAVEL_WT*(1 - distance_completion);
  }
  else if(goal == SCOUT){
    progress = TRAVEL_WT/*+ building revolution completion*/;
  }
  else if(goal == RTL || goal == LAND){ //RTL or Land
    float distance_remaining  = CalculateDistance(pos_target,pos_local);
    float distance_total      = CalculateDistance(pos_target,pos_previous);
    float distance_completion = distance_remaining/distance_total;
    progress = 1 - distance_completion;
  }

  return progress;
}

Eigen::Vector3d SimpleControl::CircleShape(int angle){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 6.0f;	// 5 meters radius

		return Eigen::Vector3d( r * (cos(angles::from_degrees(angle) - 7)),
				                    r * (sin(angles::from_degrees(angle) - 9)),
				                    pos_previous.z);
	}

void SimpleControl::Run(int uav_num)
{
  if(battery.remaining < BATTERY_MIN){
    //Return to launch site if battery is starting to get low
    goal = RTL;
  }
  else if(goal == TRAVEL){
    if(ComparePosition(pos_local, pos_target) == 0){
      //Vehicle is at target location => Scout Building
      pos_previous = pos_local;
      goal = SCOUT;
      ROS_INFO_STREAM("Scouting Building.");
    }
    else if(abs(pos_local.z - pos_target.z) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(pos_target, uav_num);
    }
    else{ //Ascend to the proper altitude first at the current location
      this->SetLocalPosition(pos_local.x, pos_local.y, pos_target.z, uav_num);
    }
  }
  else if(goal == SCOUT){
    //TODO: Fix Scout Functionality. Temporary Circle Path Test
    static int theta = 0;

	  /*tf::pointEigenToMsg(this->CircleShape(theta), pos_target); //Update Target Pos
	  this->SetLocalPosition(pos_target);
    goal = TRAVEL;
    theta++;*/

    //if (theta == 360){
      ROS_INFO_STREAM("Home Target: " << pos_home);
      pos_target = pos_home;
      goal = RTL;
      theta = 0;
    //}
  }
  else if(goal == RTL){
    if(ComparePosition(pos_local, pos_target) == 0){
      //Vehicle is at target location => Disarm
      goal = DISARM;
    }
    else if(abs(pos_local.x - pos_target.x) <= THRESHOLD_XY && abs(pos_local.y - pos_target.y) <= THRESHOLD_XY){
      this->SetLocalPosition(pos_local.x, pos_local.y, 0, uav_num);
      //goal = LAND;
    }
    else if(abs(pos_local.z - ALT_RTL) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(pos_target.x, pos_target.y, ALT_RTL, uav_num);
    }
    else{
      this->SetLocalPosition(pos_local.x, pos_local.y, ALT_RTL, uav_num);
    }
  }
  else if(goal == LAND){
    if(pos_local.z == 0){
      //Landed => Disarm
      goal = DISARM;
    }
    else{
      this->SetLocalPosition(pos_target, uav_num);
    }
  }
  else if(goal == DISARM){
    //Disarm the vehicle if it's currently armed
    if(state[uav_num].armed) this->Arm(false, uav_num);
  }
  else{
    //Wait for the goal to change
  }
}
