#include <rqt_gcs/simple_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  SimpleControl quadrotors;

  //quad1.ScoutBuilding(7,4,1);
  boost::thread_group tg;
  ros::Rate loop_rate(1); //10Hz

  while(ros::ok())
  {
    //Let the quad do it's current mission
    //quad1.Run();
    for(int i = 0; i < NUM_UAV; i++){
      ROS_INFO_STREAM("UAV " << i+1);
      ROS_INFO_STREAM("Battery:\t" << quadrotors.GetBatteryStatus(i+1).remaining);
      ROS_INFO_STREAM("Armed:\t" << (bool)(quadrotors.GetState(i+1).armed));
      ROS_INFO_STREAM("Mode:\t" << quadrotors.GetState(i+1).mode);
      ROS_INFO_STREAM("Vertical Speed:\t" << quadrotors.GetFlightState(i+1).vertical_speed);
      ROS_INFO_STREAM("Yaw:\t" << quadrotors.GetFlightState(i+1).yaw);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}

SimpleControl::SimpleControl(void)  //Class constructor
{
  for(int index = 0; index < NUM_UAV; index++){

    std::string str_uav_num = std::to_string(index+1);

    //Initialize Service Clients
    sc_arm[index]      = nh_simple_control.serviceClient<mavros_msgs::CommandBool>(uav_ns + str_uav_num + "/mavros/cmd/arming");
    sc_takeoff[index]  = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(uav_ns + str_uav_num + "/mavros/cmd/takeoff");
    sc_land[index]     = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(uav_ns + str_uav_num + "/mavros/cmd/land");
    sc_mode[index]     = nh_simple_control.serviceClient<mavros_msgs::SetMode>(uav_ns + str_uav_num + "/mavros/set_mode");
    sc_mission[index]  = nh_simple_control.serviceClient<mavros_msgs::WaypointPush>(uav_ns + str_uav_num + "/mavros/mission/push");

    //Initialize Publisher Objects
    pub_override_rc[index]       = nh_simple_control.advertise<mavros_msgs::OverrideRCIn>(uav_ns + str_uav_num + "/mavros/rc/override",QUEUE_SIZE);
    pub_setpoint_position[index] = nh_simple_control.advertise<geometry_msgs::PoseStamped>(uav_ns + str_uav_num + "/mavros/setpoint_position/local",QUEUE_SIZE);
    pub_setpoint_attitude[index] = nh_simple_control.advertise<geometry_msgs::PoseStamped>(uav_ns + str_uav_num + "/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
    pub_angular_vel[index]       = nh_simple_control.advertise<geometry_msgs::TwistStamped>(uav_ns + str_uav_num + "/mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
    pub_linear_vel[index]        = nh_simple_control.advertise<geometry_msgs::TwistStamped>(uav_ns + str_uav_num + "/mavros/setpoint_velocity/cmd_vel",QUEUE_SIZE);
    pub_setpoint_accel[index]    = nh_simple_control.advertise<geometry_msgs::Vector3Stamped>(uav_ns + str_uav_num + "/mavros/setpoint_accel/accel",QUEUE_SIZE);

    //Initialze Subscribers
    sub_state[index]      = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/state", QUEUE_SIZE, &SimpleControl::StateCallback, this);
    sub_battery[index]    = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/battery", QUEUE_SIZE, &SimpleControl::BatteryCallback, this);
    sub_imu[index]        = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/imu/data", QUEUE_SIZE, &SimpleControl::ImuCallback, this);
    sub_altitude[index]   = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/global_position/rel_alt", QUEUE_SIZE, &SimpleControl::RelAltitudeCallback, this);
    sub_heading[index]    = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/global_position/compass_hdg", QUEUE_SIZE, &SimpleControl::HeadingCallback, this);
    sub_vel[index]        = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/local_position/velocity", QUEUE_SIZE, &SimpleControl::VelocityCallback, this);
    sub_pos_global[index] = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/global_position/global", QUEUE_SIZE, &SimpleControl::NavSatFixCallback, this);
    sub_pos_local[index]  = nh_simple_control.subscribe(uav_ns + str_uav_num + "/mavros/local_position/pose", QUEUE_SIZE, &SimpleControl::LocalPosCallback, this);

    //Set Home position
    pos_home[index].x = pos_home[index].y = pos_home[index].z = 0;
  }
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
}

void SimpleControl::Arm(bool value, int uav_num)
{
  uav_num--; //For indexing arrays properly

  if(state[uav_num].armed != value){ //Only change to new state if it's different
    //Create a message for arming/disarming
    mavros_msgs::CommandBool arm;
    arm.request.value = value;

    //Call the service
    if(sc_arm[uav_num].call(arm)){
      if(arm.response.success == 1){

        bool timeout = false;
        int count = 0;
        ros::Rate check_frequency(CHECK_FREQUENCY);

        //Wait for the FCU to arm/disarm
        while((bool)state[uav_num].armed != value && !timeout){
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
  uav_num--; //For indexing arrays properly

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
  if(sc_takeoff[uav_num].call(takeoff)){
    if(takeoff.response.success == 1) ROS_INFO_STREAM("Takeoff Initiated.");
    else ROS_ERROR_STREAM("Failed to initiate takeoff.");
  }
  else{
    ROS_ERROR_STREAM("Failed to call takeoff service!");
  }
}

void SimpleControl::Land(int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create a message for landing
  mavros_msgs::CommandTOL land;

  //Call the service
  if(sc_land[uav_num].call(land)){
    if(land.response.success == 1) ROS_INFO_STREAM("Land Initiated.");
    else ROS_ERROR_STREAM("Failed to initiate land.");
  }
  else{
    ROS_ERROR_STREAM("Failed to call land service!");
  }
}

void SimpleControl::SetMode(std::string mode, int uav_num)
{
  uav_num--; //For indexing arrays properly
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
  if(sc_mode[uav_num].call(new_mode)){
    if(new_mode.response.success == 1) ROS_INFO_STREAM("Mode changed to " << mode << ".");
    else ROS_ERROR_STREAM("Failed to change flight mode to " << mode << ".");
  }
  else{
    ROS_ERROR_STREAM("Failed to call new_mode service!");
  }
}

std::string SimpleControl::GetLocation(int uav_num)
{
  uav_num--; //For indexing arrays properly

  float lat = pos_global[uav_num].latitude;
  float lon = pos_global[uav_num].longitude;

  return std::to_string(lat) + "," + std::to_string(lon);
}

void SimpleControl::ScoutBuilding(int x, int y, int z, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Update the target location
  pos_target[uav_num].x = x;
  pos_target[uav_num].y = y;
  pos_target[uav_num].z = z;

  //Prepare the vehicle for traveling to the waypoint
  //this->Arm(true);
  //this->SetMode("Guided");

  pos_previous[uav_num] = pos_local[uav_num];
  goal[uav_num] = TRAVEL;
  ROS_INFO_STREAM("Traveling to target location.");
}

void SimpleControl::OverrideRC(int channel, int value, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message object
  mavros_msgs::OverrideRCIn override_msg;

  // Update the message with the new RC value
  override_msg.channels[channel-1] = value;

  //Publish the message
  pub_override_rc[uav_num].publish(override_msg);
}

void SimpleControl::SetLocalPosition(int x, int y, int z, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  geometry_msgs::Pose point;
  point.position.x = x;
  point.position.y = y;
  point.position.z = z;
  position_stamped.pose = point;

  //Publish the message
  pub_setpoint_position[uav_num].publish(position_stamped);
}

void SimpleControl::SetLocalPosition(geometry_msgs::Point new_point, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  position_stamped.pose.position = new_point;

  //Publish the message
  pub_setpoint_position[uav_num].publish(position_stamped);
}

void SimpleControl::SetAttitude(float roll, float pitch, float yaw, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message to be published
  geometry_msgs::PoseStamped msg_pose;

  //Construct a Quaternion from Fixed angles and update pose
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  quaternionTFToMsg(q, msg_pose.pose.orientation);

  //Publish the message
  pub_setpoint_attitude[uav_num].publish(msg_pose);
}

void SimpleControl::SetAngularVelocity(int roll_vel, int pitch_vel, int yaw_vel, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message object
  geometry_msgs::TwistStamped msg_angular_vel;

  //Update the message with the new angular velocity
  geometry_msgs::Twist velocity;
  velocity.angular.x = roll_vel;
  velocity.angular.y = pitch_vel;
  velocity.angular.z = yaw_vel;
  msg_angular_vel.twist = velocity;

  //Publish the message
  pub_angular_vel[uav_num].publish(msg_angular_vel);
}

void SimpleControl::SetLinearVelocity(float x, float y, float z, int uav_num)
{
  uav_num--; //For indexing arrays properly
  geometry_msgs::TwistStamped msg_linear_vel;

  msg_linear_vel.twist.linear.x = x;
  msg_linear_vel.twist.linear.y = y;
  msg_linear_vel.twist.linear.z = z;

  pub_linear_vel[uav_num].publish(msg_linear_vel);
}

void SimpleControl::SetAcceleration(float x, float y, float z, int uav_num)
{
  uav_num--; //For indexing arrays properly

  //Create the message object
  geometry_msgs::Vector3Stamped msg_accel;

  //Update the message with the new acceleration
  msg_accel.vector.x = x;
  msg_accel.vector.y = y;
  msg_accel.vector.z = z;

  //Publish the message
  pub_setpoint_accel[uav_num].publish(msg_accel);
}

//TODO: Fix Roll, Pitch, Yaw, and Ground Speed values
FlightState SimpleControl::UpdateFlightState(int uav_num)
{
  uav_num--; //For indexing arrays properly

  struct FlightState flight_state;

  flight_state.roll = imu[uav_num].orientation.x; //Update Roll value
  flight_state.pitch = imu[uav_num].orientation.y; //Update Pitch Value
  flight_state.yaw = imu[uav_num].orientation.z; //Update Yaw Value
  flight_state.heading = heading_deg[uav_num]; //Update heading [degrees]
  flight_state.altitude = altitude_rel[uav_num]; //Update Altitude [m]
  flight_state.ground_speed = velocity[uav_num].twist.linear.x; //Global Velocity X [m/s]
  flight_state.vertical_speed = velocity[uav_num].twist.linear.z; //Global Velocity vertical [m/s]

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

float SimpleControl::GetMissionProgress(int uav_num)
{
  uav_num--; //For indexing arrays properly

  geometry_msgs::Point uav_pos_local = pos_local[uav_num];
  geometry_msgs::Point uav_pos_target = pos_target[uav_num];
  geometry_msgs::Point uav_pos_previous = pos_previous[uav_num];
  geometry_msgs::Point uav_pos_home = pos_home[uav_num];

  float progress  = 0;
  int uav_goal    = goal[uav_num];

  if(uav_goal == TRAVEL){
    float distance_remaining  = CalculateDistance(uav_pos_target,uav_pos_local);
    float distance_total      = CalculateDistance(uav_pos_target,uav_pos_home);
    float distance_completion = distance_remaining/distance_total;
    progress =  TRAVEL_WT*(1 - distance_completion);
  }
  else if(uav_goal == SCOUT){
    progress = TRAVEL_WT/*+ building revolution completion*/;
  }
  else if(uav_goal == RTL || uav_goal == LAND){ //RTL or Land
    float distance_remaining  = CalculateDistance(uav_pos_target,uav_pos_local);
    float distance_total      = CalculateDistance(uav_pos_target,uav_pos_previous);
    float distance_completion = distance_remaining/distance_total;
    progress = 1 - distance_completion;
  }

  return progress;
}

Eigen::Vector3d SimpleControl::CircleShape(int angle, int uav_num){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 6.0f;	// 5 meters radius

		return Eigen::Vector3d( r * (cos(angles::from_degrees(angle) - 7)),
				                    r * (sin(angles::from_degrees(angle) - 9)),
				                    pos_previous[uav_num].z);
	}

void SimpleControl::Run(int uav_num)
{
  geometry_msgs::Point uav_pos_local    = pos_local[uav_num-1];
  geometry_msgs::Point uav_pos_target   = pos_target[uav_num-1];
  geometry_msgs::Point uav_pos_previous = pos_previous[uav_num-1];
  geometry_msgs::Point uav_pos_home     = pos_home[uav_num-1];
  int uav_goal                          = goal[uav_num -1];

  if(battery[uav_num-1].remaining < BATTERY_MIN){
    //Return to launch site if battery is starting to get low
    uav_goal = RTL;
  }
  else if(uav_goal == TRAVEL){
    if(ComparePosition(uav_pos_local, uav_pos_target) == 0){
      //Vehicle is at target location => Scout Building
      uav_pos_previous = uav_pos_local;
      uav_goal = SCOUT;
      ROS_INFO_STREAM("Scouting Building.");
    }
    else if(abs(uav_pos_local.z - uav_pos_target.z) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(uav_pos_target, uav_num);
    }
    else{ //Ascend to the proper altitude first at the current location
      this->SetLocalPosition(uav_pos_local.x, uav_pos_local.y, uav_pos_target.z, uav_num);
    }
  }
  else if(uav_goal == SCOUT){
    //TODO: Fix Scout Functionality. Temporary Circle Path Test
    static int theta = 0;

	  /*tf::pointEigenToMsg(this->CircleShape(theta), pos_target); //Update Target Pos
	  this->SetLocalPosition(pos_target);
    uav_goal = TRAVEL;
    theta++;*/

    //if (theta == 360){
      ROS_INFO_STREAM("Home Target: " << uav_pos_home);
      uav_pos_target = uav_pos_home;
      uav_goal = RTL;
      theta = 0;
    //}
  }
  else if(uav_goal == RTL){
    if(ComparePosition(uav_pos_local, uav_pos_target) == 0){
      //Vehicle is at target location => Disarm
      uav_goal = DISARM;
    }
    else if(abs(uav_pos_local.x - uav_pos_target.x) <= THRESHOLD_XY && abs(uav_pos_local.y - uav_pos_target.y) <= THRESHOLD_XY){
      this->SetLocalPosition(uav_pos_local.x, uav_pos_local.y, 0, uav_num);
      //uav_goal = LAND;
    }
    else if(abs(uav_pos_local.z - ALT_RTL) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(uav_pos_target.x, uav_pos_target.y, ALT_RTL, uav_num);
    }
    else{
      this->SetLocalPosition(uav_pos_local.x, uav_pos_local.y, ALT_RTL, uav_num);
    }
  }
  else if(uav_goal == LAND){
    if(uav_pos_local.z == 0){
      //Landed => Disarm
      uav_goal = DISARM;
    }
    else{
      this->SetLocalPosition(uav_pos_target, uav_num);
    }
  }
  else if(uav_goal == DISARM){
    //Disarm the vehicle if it's currently armed
    if(state[uav_num-1].armed) this->Arm(false, uav_num);
  }
  else{
    //Wait for the goal to change
  }
}
