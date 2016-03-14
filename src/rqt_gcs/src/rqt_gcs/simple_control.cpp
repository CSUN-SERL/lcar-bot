#include <rqt_gcs/simple_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_control");
  SimpleControl quad1{1};

  ros::Rate loop_rate(10); //10Hz

  while(ros::ok())
  {
    quad1.Run();
    ros::spinOnce();
    loop_rate.sleep();
  }

}

int SimpleControl::id = 0;


SimpleControl::SimpleControl()  //Class constructor
{
    ++id;
    this->InitialSetup();
}

SimpleControl::SimpleControl(int uav_id)  //Class constructor
{
    id = uav_id;
    this->InitialSetup();
}

SimpleControl::~SimpleControl(void)
{
  //Class destructor
}

void SimpleControl::InitialSetup()
{
    ns = DEF_NS + std::to_string(id); //UAV Namespace

    //Initialize Service Clients
    sc_arm      = nh_simple_control.serviceClient<mavros_msgs::CommandBool>(ns + "/mavros/cmd/arming");
    sc_takeoff  = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(ns + "/mavros/cmd/takeoff");
    sc_land     = nh_simple_control.serviceClient<mavros_msgs::CommandTOL>(ns + "/mavros/cmd/land");
    sc_mode     = nh_simple_control.serviceClient<mavros_msgs::SetMode>(ns + "/mavros/set_mode");
    sc_mission  = nh_simple_control.serviceClient<mavros_msgs::WaypointPush>(ns + "/mavros/mission/push");

    //Initialize Publisher Objects
    pub_override_rc       = nh_simple_control.advertise<mavros_msgs::OverrideRCIn>(ns + "/mavros/rc/override",QUEUE_SIZE);
    pub_setpoint_position = nh_simple_control.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/setpoint_position/local",QUEUE_SIZE);
    pub_setpoint_attitude = nh_simple_control.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
    pub_angular_vel       = nh_simple_control.advertise<geometry_msgs::TwistStamped>(ns + "/mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
    pub_linear_vel        = nh_simple_control.advertise<geometry_msgs::TwistStamped>(ns + "/mavros/setpoint_velocity/cmd_vel",QUEUE_SIZE);
    pub_setpoint_accel    = nh_simple_control.advertise<geometry_msgs::Vector3Stamped>(ns + "/mavros/setpoint_accel/accel",QUEUE_SIZE);

    //Initialze Subscribers
    sub_state      = nh_simple_control.subscribe(ns + "/mavros/state", QUEUE_SIZE, &SimpleControl::StateCallback, this);
    sub_battery    = nh_simple_control.subscribe(ns + "/mavros/battery", QUEUE_SIZE, &SimpleControl::BatteryCallback, this);
    sub_imu        = nh_simple_control.subscribe(ns + "/mavros/imu/data", QUEUE_SIZE, &SimpleControl::ImuCallback, this);
    sub_altitude   = nh_simple_control.subscribe(ns + "/mavros/global_position/rel_alt", QUEUE_SIZE, &SimpleControl::RelAltitudeCallback, this);
    sub_heading    = nh_simple_control.subscribe(ns + "/mavros/global_position/compass_hdg", QUEUE_SIZE, &SimpleControl::HeadingCallback, this);
    sub_vel        = nh_simple_control.subscribe(ns + "/mavros/local_position/velocity", QUEUE_SIZE, &SimpleControl::VelocityCallback, this);
    sub_pos_global = nh_simple_control.subscribe(ns + "/mavros/global_position/global", QUEUE_SIZE, &SimpleControl::NavSatFixCallback, this);
    sub_pos_local  = nh_simple_control.subscribe(ns + "/mavros/local_position/pose", QUEUE_SIZE, &SimpleControl::LocalPosCallback, this);

    //Set Home position
    pose_home.position.x = pose_home.position.y = pose_home.position.z = 0;
}

void SimpleControl::Arm(bool value)
{
  if(state.armed != value){ //Only change to new state if it's different
    //Create a message for arming/disarming
    mavros_msgs::CommandBool arm;
    arm.request.value = value;
      //Call the service

    if(pose_local.position.z > 1 && !value){
      this->SetMode("AUTO.LAND");
    }
    else if(sc_arm.call(arm)){
      if(arm.response.success == 1){

        bool timeout = false;
        int count = 0;
        ros::Rate check_frequency(CHECK_FREQUENCY);

        //Wait for the FCU to arm/disarm
        while((bool)state.armed != value && !timeout){
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
          if(state.armed) ROS_INFO_STREAM("**ARMED**");
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

void SimpleControl::Takeoff(int altitude)
{
  //Ensure the UAV is in Guided mode and armed
  bool armed = (bool)state.armed;
  std::string mode = state.mode;

  if(mode.compare("GUIDED") != 0) this->SetMode("Guided");
  if(!armed) this->Arm(true);

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

void SimpleControl::Land()
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

void SimpleControl::SetMode(std::string mode)
{
  //Create a message for changing flight mode
  mavros_msgs::SetMode new_mode;
  new_mode.request.base_mode = 0;
  new_mode.request.custom_mode = mode; //custom_mode expects a char*

  //Call the service
  if(sc_mode.call(new_mode)){
    if(new_mode.response.success == 1) ROS_INFO_STREAM("Mode changed to " << mode << ".");
    else ROS_ERROR_STREAM("Failed to change flight mode to " << mode << ".");
  }
  else{
    ROS_ERROR_STREAM("Failed to call new_mode service!");
  }
}

void SimpleControl::EnableOffboard()
{
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = pose_local.position.x;
  pose.pose.position.y = pose_local.position.y;
  pose.pose.position.z = pose_local.position.z;

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  if( state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){

      ros::Rate loop_rate(50); //50Hz
      //send a few setpoints before starting
      for(int i = 10; ros::ok() && i > 0; --i){
          pub_setpoint_position.publish(pose);
          ros::spinOnce();
          loop_rate.sleep();
      }

      if( sc_mode.call(offb_set_mode) && offb_set_mode.response.success){
          ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
  } else {
      if( !state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
          if( sc_arm.call(arm_cmd) && arm_cmd.response.success){
              ROS_INFO("Vehicle armed");
          }
          last_request = ros::Time::now();
      }
  }
}

std::string SimpleControl::GetLocation()
{
  float lat = pos_global.latitude;
  float lon = pos_global.longitude;

  return std::to_string(lat) + "," + std::to_string(lon);
}

void SimpleControl::ScoutBuilding(float x, float y, float z)
{
  this->EnableOffboard();
  //Update the target location
  pose_target.position.x = x;
  pose_target.position.y = y;
  pose_target.position.z = z;

  pose_previous = pose_local;
  goal = travel;
  ROS_INFO_STREAM("Traveling to target location.");
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
  wp1.z_alt        = 585;

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
    ROS_INFO_STREAM("Response from service: " << msg_mission.response);
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

void SimpleControl::SetLocalPosition(float x, float y, float z, float yaw = 0)
{
  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  position_stamped.pose.position.x      = x;
  position_stamped.pose.position.y      = y;
  position_stamped.pose.position.z      = z;
  position_stamped.pose.orientation.z   = yaw;

  //Publish the message
  pub_setpoint_position.publish(position_stamped);
}

void SimpleControl::SetLocalPosition(geometry_msgs::Pose new_pose)
{
  //Create the message object
  geometry_msgs::PoseStamped position_stamped;

  //Update the message with the new position
  position_stamped.pose = new_pose;

  //Publish the message
  pub_setpoint_position.publish(position_stamped);
}

void SimpleControl::SetAttitude(float roll, float pitch, float yaw)
{
  //Create the message to be published
  geometry_msgs::PoseStamped msg_pose;

  //Construct a Quaternion from Fixed angles and update pose
  tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
  quaternionTFToMsg(q, msg_pose.pose.orientation);

  //Publish the message
  pub_setpoint_attitude.publish(msg_pose);
}

void SimpleControl::SetAngularVelocity(float roll_vel, float pitch_vel, float yaw_vel)
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

void SimpleControl::SetLinearVelocity(float x, float y, float z)
{
  geometry_msgs::TwistStamped msg_linear_vel;

  msg_linear_vel.twist.linear.x = x;
  msg_linear_vel.twist.linear.y = y;
  msg_linear_vel.twist.linear.z = z;

  pub_linear_vel.publish(msg_linear_vel);
}

void SimpleControl::SetAcceleration(float x, float y, float z)
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
FlightState SimpleControl::UpdateFlightState()
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

int SimpleControl::ComparePosition(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  int result;

  if( abs(pose2.position.x - pose1.position.x) <= THRESHOLD_XY  &&
      abs(pose2.position.y - pose1.position.y) <= THRESHOLD_XY  &&
      abs(pose2.position.z - pose1.position.z) <= THRESHOLD_Z   /*&&
      abs(pose2.orientation.z - pose1.orientation.z) <= THRESHOLD_YAW*/){
    result = 0;
  }
  else result = 1;

  return 1;
}

int SimpleControl::CalculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
  float dist_x = pose2.position.x - pose1.position.x;
  float dist_y = pose2.position.y - pose1.position.y;
  float dist_z = pose2.position.z - pose1.position.z;
  return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
}

float SimpleControl::GetMissionProgress()
{
  float progress  = 0;

  if(goal == travel){
    float distance_remaining  = CalculateDistance(pose_target,pose_local);
    float distance_total      = CalculateDistance(pose_target,pose_home);
    float distance_completion = distance_remaining/distance_total;
    progress =  TRAVEL_WT*(1 - distance_completion);
  }
  else if(goal == scout){
    progress = TRAVEL_WT/*+ building revolution completion*/;
  }
  else if(goal == rtl || goal == land){ //RTL or Land
    float distance_remaining  = CalculateDistance(pose_target,pose_local);
    float distance_total      = CalculateDistance(pose_target,pose_previous);
    float distance_completion = distance_remaining/distance_total;
    progress = 1 - distance_completion;
  }

  return progress;
}

geometry_msgs::Pose SimpleControl::CircleShape(int angle){
		/** @todo Give possibility to user define amplitude of movement (circle radius)*/
		double r = 5.0f;	// 5 mete;rs radius

        geometry_msgs::Pose new_pose;
        tf::pointEigenToMsg(Eigen::Vector3d(r * (cos(angles::from_degrees(angle))),
                                            r * (sin(angles::from_degrees(angle))),
                                            pose_previous.position.z),
                                            new_pose.position);
        new_pose.orientation.z = sin(angles::from_degrees(angle));

        return new_pose;
}

geometry_msgs::Pose SimpleControl::DiamondShape(int index){

  double r = 5.0f;
  geometry_msgs::Pose mission[4] = { };

  ROS_INFO_STREAM("Traveling to Index " << index);
  mission[0].position.x     = r;
  mission[0].position.y     = 0;
  mission[0].position.z     = 4;
  //mission[0].orientation.z  = 180;

  mission[1].position.x     = 0;
  mission[1].position.y     = r;
  mission[1].position.z     = 4;
  //mission[1].orientation.z  = 270;

  mission[2].position.x     = -r;
  mission[2].position.y     = 0;
  mission[2].position.z     = 4;
  //mission[2].orientation.z  = 0;

  mission[3].position.x     = 0;
  mission[3].position.y     = -r;
  mission[3].position.z     = 4;
  //mission[3].orientation.z  = 90;

  return mission[index];
}

void SimpleControl::Run()
{
  /*if(battery.remaining < BATTERY_MIN){
    //Return to launch site if battery is starting to get low
    goal = RTL;
  }*/
  if(goal == travel){
    if(ComparePosition(pose_local, pose_target) == 0){
      //Vehicle is at target location => Scout Building
      //pose_previous = pose_local;
      //goal = scout;
      pose_target = pose_home;
      goal = land;
      ROS_DEBUG_STREAM_ONCE("Scouting Building.");
    }
    else if(abs(pose_local.position.z - pose_target.position.z) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(pose_target);
    }
    else{
      //Ascend to the proper altitude first at the current location
      this->SetLocalPosition(pose_local.position.x, pose_local.position.y, pose_target.position.z);
    }
  }
  else if(goal == scout){
    //TODO: Fix Scout Functionality. Temporary Circle Path Test
    static int rev_count = 0;
    static int cur_point = 0;
    pose_target = this->DiamondShape(cur_point);
	  //tf::pointEigenToMsg(this->CircleShape(theta), pos_target); //Update Target Pos
      this->SetLocalPosition(pose_target);
    goal = travel;
    cur_point++;

    if (cur_point > 3){
      //ROS_INFO_STREAM("Home Target: " << pos_home);
      //pos_target = pos_home;
      //goal = RTL;
      ROS_INFO_STREAM("Circled Building" << rev_count << "times.");
      rev_count++;
      cur_point = 1;
    }
  }
  else if(goal == rtl){
    if(ComparePosition(pose_local, pose_target) == 0){
      //Vehicle is at target location => Disarm
      goal = disarm;
    }
    else if(abs(pose_local.position.x - pose_target.position.x) <= THRESHOLD_XY && abs(pose_local.position.y - pose_target.position.y) <= THRESHOLD_XY){
      this->SetLocalPosition(pose_local.position.x, pose_local.position.y, 0);
      goal = land;
    }
    else if(abs(pose_local.position.z - ALT_RTL) <= THRESHOLD_Z){
      //Achieved the proper altitude => Go to target location
      this->SetLocalPosition(pose_target.position.x, pose_target.position.y, ALT_RTL);
    }
    else{
      this->SetLocalPosition(pose_local.position.x, pose_local.position.y, ALT_RTL);
    }
    /*if(state.mode.compare("AUTO.RTL") != 0 && state.mode.compare("AUTO.LAND") != 0){
      this->SetMode("AUTO.RTL");
    }
    */
  }
  else if(goal == land){
    this->SetMode("AUTO.LAND");
    goal = idle;
  }
  else if(goal == disarm){
    //Disarm the vehicle if it's currently armed
    if(state.armed) this->Arm(false);
    goal = idle;
  }
  else if(goal == idle){
    //Wait for the goal to change
  }
}
