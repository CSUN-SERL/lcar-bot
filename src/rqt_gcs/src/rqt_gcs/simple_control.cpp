#include <rqt_gcs/simple_control.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_control");
    SimpleControl quad1{1};

    ros::Rate loop_rate(10); //10Hz

    lcar_msgs::Target target_pt;
    target_pt.target_point.position.x = 0;
    target_pt.target_point.position.y = 0;
    target_pt.target_point.position.z = 2;
    target_pt.radius = 2;

    quad1.Arm(true);
    quad1.ScoutBuilding(target_pt);

    while(ros::ok())
    {
        quad1.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

int SimpleControl::static_id = 0;


SimpleControl::SimpleControl()  //Class constructor
{
    id = static_id++;
    this->InitialSetup();
}

SimpleControl::SimpleControl(int uav_id)  //Class constructor
{
    id = uav_id;
    static_id++;
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
    sc_arm      = nh.serviceClient<mavros_msgs::CommandBool>(ns + "/mavros/cmd/arming");
    sc_takeoff  = nh.serviceClient<mavros_msgs::CommandTOL>(ns + "/mavros/cmd/takeoff");
    sc_land     = nh.serviceClient<mavros_msgs::CommandTOL>(ns + "/mavros/cmd/land");
    sc_mode     = nh.serviceClient<mavros_msgs::SetMode>(ns + "/mavros/set_mode");
    sc_mission  = nh.serviceClient<mavros_msgs::WaypointPush>(ns + "/mavros/mission/push");

    //Initialize Publisher Objects
    pub_override_rc       = nh.advertise<mavros_msgs::OverrideRCIn>(ns + "/mavros/rc/override",QUEUE_SIZE);
    pub_setpoint_position = nh.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/setpoint_position/local",QUEUE_SIZE);
    pub_setpoint_attitude = nh.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/setpoint_attitude/attitude",QUEUE_SIZE);
    pub_angular_vel       = nh.advertise<geometry_msgs::TwistStamped>(ns + "/mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
    pub_linear_vel        = nh.advertise<geometry_msgs::TwistStamped>(ns + "/mavros/setpoint_velocity/cmd_vel",QUEUE_SIZE);
    pub_setpoint_accel    = nh.advertise<geometry_msgs::Vector3Stamped>(ns + "/mavros/setpoint_accel/accel",QUEUE_SIZE);
    pub_door_answer       = nh.advertise<lcar_msgs::Door>(ns + "/object_detection/door/answer",QUEUE_SIZE);
    pub_heartbeat         = nh.advertise<std_msgs::Int32>(ns + "/hearbeat/gcs", 0);
    
    //Initialize Subscribers
    sub_state      = nh.subscribe(ns + "/mavros/state", QUEUE_SIZE, &SimpleControl::StateCallback, this);
    sub_battery    = nh.subscribe(ns + "/mavros/battery", QUEUE_SIZE, &SimpleControl::BatteryCallback, this);
    sub_imu        = nh.subscribe(ns + "/mavros/imu/data", QUEUE_SIZE, &SimpleControl::ImuCallback, this);
    sub_altitude   = nh.subscribe(ns + "/mavros/global_position/rel_alt", QUEUE_SIZE, &SimpleControl::RelAltitudeCallback, this);
    sub_heading    = nh.subscribe(ns + "/mavros/global_position/compass_hdg", QUEUE_SIZE, &SimpleControl::HeadingCallback, this);
    sub_vel        = nh.subscribe(ns + "/mavros/local_position/velocity", QUEUE_SIZE, &SimpleControl::VelocityCallback, this);
    sub_pos_global = nh.subscribe(ns + "/mavros/global_position/global", QUEUE_SIZE, &SimpleControl::NavSatFixCallback, this);
    sub_pos_local  = nh.subscribe(ns + "/mavros/local_position/pose", QUEUE_SIZE, &SimpleControl::LocalPosCallback, this);
    sub_depth      = nh.subscribe(ns + "/object_avoidance/depth", QUEUE_SIZE, &SimpleControl::DepthCallback, this);
    //sub_door_query = nh.subscribe(ns + "/objectdetection/door/query", QUEUE_SIZE, &SimpleControl::DoorQueryCallback, this);
    sub_detection  = nh.subscribe(ns + "/object_detection/access_point/door", QUEUE_SIZE, &SimpleControl::DetectionCallback, this);
    sub_heartbeat  = nh.subscribe(ns + "/heartbeat/uav", 0, &SimpleControl::UavHeartbeatCallback, this); 
    
    uav_heartbeat_timer = nh.createTimer(ros::Duration(0.25), &SimpleControl::UavHeartbeatTimeoutCallback, this);
    gcs_heartbeat_timer = nh.createTimer(ros::Duration(0.1), &SimpleControl::GcsPublishHeartbeat, this);        
    
    //Set Home position
    pose_home.position.x = pose_home.position.y = pose_home.position.z = 0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose_home.orientation);

    object_distance.data = 100;
    battery.remaining = -1;
}

void SimpleControl::Arm(bool value)
{
    if(state.armed != value){ //Only change to new state if it's different

        //Create a message for arming/disarming
        mavros_msgs::CommandBool arm;
        arm.request.value = value;

        //Call the service
        if(pose_local.position.z > 1 && !value){
            ROS_ERROR_STREAM("Cannot disarm in air! Landing.");
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
        ROS_INFO_STREAM("Mode: " << mode);
        ROS_ERROR_STREAM("Failed to call change flight mode service!");
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

void SimpleControl::ScoutBuilding(lcar_msgs::Target msg_target)
{
    this->EnableOffboard();
    //Update the target location
    path_mission = CircleShape(msg_target);
    pose_target = path_mission.poses.at(0).pose;
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

void SimpleControl::SetLocalPosition(float x, float y, float z, float yaw)
{
    //Create the message object
    geometry_msgs::PoseStamped position_stamped;

    //Update the message with the new position
    position_stamped.pose.position.x = x;
    position_stamped.pose.position.y = y;
    position_stamped.pose.position.z = z;
    if(yaw == 361){ //Default or invalid value passed, use current Yaw value
        position_stamped.pose.orientation = pose_previous.orientation;
    }
    else{ //Use the specified Yaw value
        quaternionTFToMsg(tf::createQuaternionFromYaw(yaw*(PI/180)), position_stamped.pose.orientation);
    }

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

    /*tf::Quaternion quaternion_tf;
  tf::quaternionMsgToTF(imu.orientation, quaternion_tf);
  tf::Matrix3x3 m{quaternion_tf};

  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);*/

    flight_state.roll = imu.orientation.x;     //Update Roll value
    flight_state.pitch = imu.orientation.y;   //Update Pitch Value
    flight_state.yaw = imu.orientation.z;       //Update Yaw Value
    flight_state.heading = heading_deg.data;  //Update heading [degrees]
    flight_state.altitude = altitude_rel.data;//Update Altitude [m]
    flight_state.ground_speed = velocity.twist.linear.x;  //Global Velocity X [m/s]
    flight_state.vertical_speed = velocity.twist.linear.z;//Global Velocity vertical [m/s]

    return flight_state;
}

  int SimpleControl::ComparePosition(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    int result;

    //Get the yaw values
    double roll, pitch, yaw1, yaw2;
    tf::Quaternion quaternion_tf1, quaternion_tf2;

    tf::quaternionMsgToTF(pose1.orientation, quaternion_tf1);
    tf::Matrix3x3 m1{quaternion_tf1};

    tf::quaternionMsgToTF(pose2.orientation, quaternion_tf2);
    tf::Matrix3x3 m2{quaternion_tf2};

    m1.getRPY(roll, pitch, yaw1);
    m2.getRPY(roll, pitch, yaw2);

    if( std::abs(pose2.position.x - pose1.position.x)   <= THRESHOLD_XY &&
        std::abs(pose2.position.y - pose1.position.y)   <= THRESHOLD_XY &&
        std::abs(pose2.position.z - pose1.position.z)   <= THRESHOLD_Z  &&
        std::abs(yaw2 - yaw1)                           <= THRESHOLD_YAW){
        result = 0;
    }
    else result = 1;

    return result;
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

nav_msgs::Path SimpleControl::CircleShape(lcar_msgs::Target target_point){
    geometry_msgs::PoseStamped pose_new_stamped;
    geometry_msgs::Pose pose_new;
    geometry_msgs::Point point_center = target_point.target_point.position;
    nav_msgs::Path mission;
    float yaw_angle, radius = target_point.radius;

    //Generate the Mission
    for(int angle = 0; angle < 360; angle++){
        //Generate a position from the angle
        tf::pointEigenToMsg(Eigen::Vector3d(radius * (cos(angles::from_degrees(angle))),
                                            radius * (sin(angles::from_degrees(angle))),
                                            point_center.z),
                            pose_new.position);
        //Offset the center point to the target location
        pose_new.position.x += point_center.x;
        pose_new.position.y += point_center.y;

        //Create the yaw angle that points to the center of the circle
        yaw_angle = angles::normalize_angle_positive(angles::from_degrees(angle + 180));
        quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle), pose_new.orientation);

        pose_new_stamped.pose = pose_new;
        mission.poses.push_back(pose_new_stamped);
    }

    return mission;
}

nav_msgs::Path SimpleControl::DiamondShape(lcar_msgs::Target target_point)
{
    nav_msgs::Path mission;
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::Pose pose;

    pose.position.x    += target_point.radius;
    pose.position.y     = target_point.target_point.position.y;
    pose.position.z     = target_point.target_point.position.z;
    quaternionTFToMsg(tf::createQuaternionFromYaw(angles::from_degrees(180)), pose.orientation);
    pose_stamped.pose = pose;
    mission.poses.push_back(pose_stamped);

    pose.position.x     = target_point.target_point.position.x;
    pose.position.y    += target_point.radius;
    pose.position.z     = target_point.target_point.position.z;
    quaternionTFToMsg(tf::createQuaternionFromYaw(angles::from_degrees(270)), pose.orientation);
    pose_stamped.pose = pose;
    mission.poses.push_back(pose_stamped);

    pose.position.x    -= target_point.radius;
    pose.position.y     = target_point.target_point.position.y;
    pose.position.z     = target_point.target_point.position.z;
    quaternionTFToMsg(tf::createQuaternionFromYaw(angles::from_degrees(0)), pose.orientation);
    pose_stamped.pose = pose;
    mission.poses.push_back(pose_stamped);

    pose.position.x     = target_point.target_point.position.x;
    pose.position.y    -= target_point.radius;
    pose.position.z     = target_point.target_point.position.z;
    quaternionTFToMsg(tf::createQuaternionFromYaw(angles::from_degrees(90)), pose.orientation);

    pose_stamped.pose = pose;
    mission.poses.push_back(pose_stamped);

    return mission;
}

void SimpleControl::Run()
{
    //Sanity Checks
    if(battery.remaining < BATTERY_MIN && battery.remaining != -1){
        //Land if battery is starting to get low
        goal = land;
    }

    if(object_distance.data < THRESHOLD_DEPTH){
        //Collision Imminent! Land.
        ROS_ERROR_STREAM_DELAYED_THROTTLE(5, "Collision Imminent!");
        if(goal != hold_pose){
            pose_previous = pose_local;
            goal_prev = goal;
            goal = hold_pose;
        }
        collision = true;
    }
    else if(goal_prev != null && collision){
        goal = goal_prev;
        collision = false;
    }

    if(goal == travel){
        if(ComparePosition(pose_local, pose_target) == 0){
            //Vehicle is at target location => Scout Building
            pose_previous = pose_local;
            goal = scout;
            ROS_DEBUG_STREAM_ONCE("Scouting Building.");
        }
        else if(std::abs(std::abs(pose_local.position.z) - std::abs(pose_target.position.z)) <= THRESHOLD_Z){
            //Achieved the proper altitude => Go to target location
            this->SetLocalPosition(pose_target);

            pose_previous = pose_local; //Update previous position for fixing the altitude
        }
        else{
            //Ascend to the proper altitude first at the current location
            this->SetLocalPosition(pose_previous.position.x, pose_previous.position.y, pose_target.position.z);
        }
    }
    else if(goal == hold_pose){
        this->SetLocalPosition(pose_previous);
    }
    else if(goal == scout){
        static int rev_count = 0;
        static int cur_point = 0;

        //Travel to the next waypoint
        pose_target = path_mission.poses.at(cur_point).pose;
        this->SetLocalPosition(pose_target);
        goal = travel;
        cur_point++;

        if (cur_point > path_mission.poses.size()-1){ //
            rev_count++;
            ROS_INFO_STREAM("Circled Building " << rev_count << " times.");

            if(rev_count > 2) {
                goal = rtl;
                rev_count = 0;
                cur_point = 0;
            }
            else cur_point = 0;
        }
    }
    else if(goal == rtl){
        if(ComparePosition(pose_local, pose_home) == 0){
            goal = land;
        }
        else if(abs(pose_local.position.z - ALT_RTL) <= THRESHOLD_Z){
            //Achieved the proper altitude => Go to target location
            this->SetLocalPosition(pose_home.position.x, pose_home.position.y, ALT_RTL);
            pose_previous = pose_local;
        }
        else{
            this->SetLocalPosition(pose_previous.position.x, pose_previous.position.y, ALT_RTL);
        }
    }
    else if(goal == land){
        if(pose_local.position.z <= THRESHOLD_Z){
            goal = disarm;
            ROS_INFO_STREAM_ONCE("Landed Safely.");
        }
        else{
            //Descend to the floor
            this->SetLocalPosition(pose_previous.position.x, pose_previous.position.y, 0);
        }
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
