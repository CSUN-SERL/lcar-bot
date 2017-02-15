
#include "vehicle/uav_control.h"

namespace gcs
{

UAVControl::UAVControl(int uav_id) : MavrosHelper(uav_id)
{
    this->InitialSetup();
}

UAVControl::~UAVControl()
{
    //Class destructor
}

void UAVControl::InitialSetup()
{
    ros::NodeHandle nh("V" + std::to_string(id));

    pub_door_answer = nh.advertise<lcar_msgs::Query>("object_detection/door/answer", QUEUE_SIZE);
    pub_heartbeat   = nh.advertise<std_msgs::Int32>("heartbeat/gcs", 0);

    //Initialize Subscribers
    sub_depth       = nh.subscribe("object_avoidance/depth", QUEUE_SIZE, &UAVControl::DepthCallback, this);
    sub_detection  = nh.subscribe("object_detection/access_point/door", QUEUE_SIZE, &UAVControl::DetectionCallback, this);
    sub_heartbeat  = nh.subscribe("heartbeat/uav", 0, &UAVControl::UavHeartbeatCallback, this);

    timer_heartbeat_uav = nh.createTimer(ros::Duration(0.25), &UAVControl::UavHeartbeatTimeoutCallback, this);
    timer_heartbeat_gcs = nh.createTimer(ros::Duration(0.1), &UAVControl::GcsPublishHeartbeat, this);

    //Set Home position
    pose_home.position.x = pose_home.position.y = pose_home.position.z = 0;
    quaternionTFToMsg(tf::createQuaternionFromYaw(0), pose_home.orientation);

    object_distance.data = 100;
    battery.percentage = -1;
}

void UAVControl::ScoutBuilding(lcar_msgs::TargetLocal msg_target)
{
    //Update the target location
    path_mission = CircleShape(msg_target);
    pose_target = path_mission.poses.at(0).pose;

    pose_previous = pose_local;
    this->EnableOffboard();

    goal = travel;
    mission_mode = active;
    ROS_INFO_STREAM("Traveling to target location.");
}

void UAVControl::ScoutBuilding(lcar_msgs::TargetGlobal msg_target)
{
    this->SendMission(CircleShape(msg_target));
    this->Arm(true);
    this->SetMode("AUTO.MISSION");

    mission_mode = active;
    position_mode = global;
}

//TODO: Fix Roll, Pitch, Yaw, and Ground Speed values
FlightState UAVControl::UpdateFlightState()
{
    FlightState flight_state;


    flight_state.roll = imu.orientation.x;     //Update Roll value
    flight_state.pitch = imu.orientation.y;   //Update Pitch Value
    flight_state.yaw = imu.orientation.z;       //Update Yaw Value
    flight_state.heading = heading_deg.data;  //Update heading [degrees]
    flight_state.altitude = altitude_rel.data;//Update Altitude [m]
    flight_state.ground_speed = velocity.twist.linear.x;  //Global Velocity X [m/s]
    flight_state.vertical_speed = velocity.twist.linear.z;//Global Velocity vertical [m/s]

    return flight_state;
}

int UAVControl::ComparePosition(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
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
        std::abs(pose2.position.y - pose1.position.y)   <= THRESHOLD_XY ){
        
        result = 0;
    }
    else result = 1;

    return result;
}

//returns angle in degrees
double UAVControl::GetYaw(geometry_msgs::Pose& pose)
{
    double roll, pitch, yaw;
    tf::Quaternion quaternion_tf;

    tf::quaternionMsgToTF(pose.orientation, quaternion_tf);
    tf::Matrix3x3 m1{quaternion_tf};
    m1.getRPY(roll, pitch, yaw);
    
    return yaw;
}

int UAVControl::CompareYaw(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    int result;
    
    //compares two radians
    if(std::abs(GetYaw(pose2) - GetYaw(pose1)) <= THRESHOLD_YAW){
        
        result = 0;
    }
    else {
        result = 1;
    }

    return result;
}

int UAVControl::CompareYaw(double yaw1, double yaw2)
{
    int result;
    
    //compares two radians
    if(std::abs(yaw2 - yaw1) <= THRESHOLD_YAW){
        
        result = 0;
    }
    else {
        result = 1;
    }

    return result;
}

int UAVControl::CompareAltitude(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    int result;
    float threshold;

//    if(position_mode == local) threshold = THRESHOLD_Z;
//    else threshold = THRESHOLD_ALT;

    if(std::abs(pose1.position.z - pose2.position.z) <= THRESHOLD_Z){
        result = 0;
    }
    else result = 1;

    return result;
}

int UAVControl::CalculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    float dist_x = pose2.position.x - pose1.position.x;
    float dist_y = pose2.position.y - pose1.position.y;
    float dist_z = pose2.position.z - pose1.position.z;
    return sqrt((dist_x * dist_x) + (dist_y * dist_y) + (dist_z * dist_z));
}

float UAVControl::GetMissionProgress()
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


void UAVControl::TravelToLocation(geometry_msgs::Pose& target)
{ 
   
    ROS_INFO_STREAM_ONCE("Traveling to Location");
    if(CompareAltitude(pose_local,pose_target) == 0)
    {     
        //Achieved the proper altitude => Go to target location
        this->PublishPosition(pose_target);
        //Update previous position for fixing the altitude
        pose_previous = pose_local;
        
    }
    else{
          ROS_INFO_STREAM_ONCE("Ascending");
        //Ascend to the proper altitude first at the current location
        this->PublishPosition(pose_previous.position.x, pose_previous.position.y, pose_target.position.z,GetYaw(pose_target));
    } 
}

void UAVControl::SetTarget(geometry_msgs::Pose& target)
{
    pose_target.position.x = target.position.x;
    pose_target.position.y = target.position.y;
    pose_target.position.z = target.position.z;
    pose_target.orientation = target.orientation;
}

void UAVControl::TravelToPosition(double x, double y)
{
    pose_target.position.x = x;
    pose_target.position.y = y;
    if(CompareAltitude(pose_local,pose_target) == 0)
    {
        if(ComparePosition(pose_local,pose_target) == 0)
        {     
            this->TravelToLocation(pose_previous);
        }
        else
        {
            this->TravelToLocation(pose_target);
        }
    }
    else
    {
        this->TravelToAltitude(pose_target.position.z);
    }
}
    
void UAVControl::TravelToAltitude(double z)
{
    pose_target.position.z = z;
    if(CompareAltitude(pose_local,pose_target) == 0)
    {
        this->TravelToLocation(pose_previous);
    }
    else
    {
        this->TravelToLocation(pose_target);
    } 
}
  
void UAVControl::TurnToAngle(double target_angle)
{
    target_angle = angles::normalize_angle_positive(angles::from_degrees(target_angle));
    quaternionTFToMsg(tf::createQuaternionFromYaw(target_angle), pose_target.orientation);
    
    if(CompareYaw((float)GetYaw(pose_local),(float)target_angle) == 0 )
    {  
        this->TravelToLocation(pose_previous);
    }
    else
    {
        this->TravelToLocation(pose_target);
    }
}

/*should only be called once*/
void UAVControl::TravelRelativeToPosition(double x,double y)
{
    pose_target.position.x += x;
    pose_target.position.y +=y;
    this->TravelToPosition(pose_target.position.x,pose_target.position.y);
}

void UAVControl::TurnRelative(double degrees_turn)
{
    degrees_turn = angles::normalize_angle_positive(angles::from_degrees(degrees_turn));
    degrees_turn += GetYaw(pose_target);
    quaternionTFToMsg(tf::createQuaternionFromYaw(degrees_turn), pose_target.orientation);
}

void UAVControl::TravelRelativeToAltitude(double z)
{
    pose_target.position.z += z;
}

void UAVControl::StrafeX(double x)
{
    pose_target.position.x +=x;
    this->TravelToPosition(pose_target.position.x,pose_target.position.y);
}

void UAVControl::StrafeY(double y)
{
    pose_target.position.y +=y;
    this->TravelToPosition(pose_target.position.x,pose_target.position.y);
}
    
nav_msgs::Path UAVControl::CircleShape(lcar_msgs::TargetLocal target_point)
{
    double yaw_angle, radius = target_point.radius;
    geometry_msgs::PoseStamped  pose_new_stamped;
    geometry_msgs::Pose         pose_new;
    geometry_msgs::Point        point_center = target_point.target.position;
    nav_msgs::Path              mission;

    //Generate the Mission
    for(int angle = 0; angle <= 360; angle++){
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

mavros_msgs::WaypointPush UAVControl::CircleShape(lcar_msgs::TargetGlobal target_point)
{
    double  yaw_angle, radius = target_point.radius;
    double  lat_center = angles::from_degrees(target_point.target.latitude),
            lon_center = angles::from_degrees(target_point.target.longitude);

    mavros_msgs::Waypoint waypoint;
    mavros_msgs::WaypointPush mission;

    waypoint.frame        = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
    waypoint.command      = mavros_msgs::CommandCode::NAV_WAYPOINT;
    waypoint.is_current   = false;
    waypoint.autocontinue = true;

    //Generate circle shape using GPS coordinates
    for(int angle = 0; angle <= 360; angle+=5){
        double lat_new = asin(sin(lat_center)*cos(radius/R_EARTH) + cos(lat_center)*sin(radius/R_EARTH)*cos(angles::from_degrees(angle)));
        double lon_new = lon_center + atan2(sin(angles::from_degrees(angle))*sin(radius/R_EARTH)*cos(lat_center),
                                           cos(radius/R_EARTH)-sin(lat_center)*sin(lat_new));

        waypoint.x_lat   = angles::to_degrees(lat_new);
        waypoint.y_long  = angles::to_degrees(lon_new);
        waypoint.z_alt   = target_point.target.altitude;

        //Create the yaw angle that points to the center of the circle
        waypoint.param4 = angles::to_degrees(angles::normalize_angle_positive(angles::from_degrees(angle + 180)));

        mission.request.waypoints.push_back(waypoint);
    }

    return mission;
}

void UAVControl::SafetyCheck()
{
    //Sanity Checks
    if(battery.percentage < BATTERY_MIN && battery.percentage != -1){
        //Land if battery is starting to get low
        goal = land;
    }

    if(object_distance.data < THRESHOLD_DEPTH){
        //Collision Imminent! Land.
        ROS_ERROR_STREAM_DELAYED_THROTTLE(5, "Collision Imminent!");
        if(goal != hold){
            pose_previous = pose_local;
            goal_prev = goal;
            goal = hold;
        }
        collision = true;
    }
    else if(goal_prev != null && collision){
        goal = goal_prev;
        collision = false;
    }
}

void UAVControl::Run()
{
    //Sanity Checks
    this->SafetyCheck();

    //Run Specific Actions
    if(position_mode == local) 
    {ROS_INFO_STREAM_ONCE("Beginning local");
    this->RunLocal();}
    else this->RunGlobal();

    //Run Common Actions
    if(goal == disarm){
        //Disarm the vehicle if it's currently armed
        if(state.armed) this->Arm(false);
    //pose_target.position.z = 2;
        goal = idle;
    }
    
    else if(goal == idle){
        //Wait for the goal to change
    }
}

void UAVControl::RunLocal()
{ 
    if(CompareAltitude(pose_local,pose_target) == 0 && ComparePosition(pose_local,pose_target)==0)
    {
        ROS_INFO_STREAM("current 1"<<pose_local.position.z);
        
        ROS_INFO_STREAM("target 1"<<pose_target.position.z);
        //this->TurnToAngle(180);
        //this->TravelToAltitude(7);
       // this->TravelToPosition(5,5);
       this->TravelToPosition(-2, -2);
        
        ROS_INFO_STREAM("current 2"<<pose_local.position.z);
        
        ROS_INFO_STREAM("target 2"<<pose_target.position.z);
        
    }
    else
    { 
        this->TravelToAltitude(5);
        this->TravelToPosition(5,5);
        
    }
    /* 
        case scout:
            static int rev_count = 1;
            static int cur_point = 0;

            //Travel to the next waypoint
            pose_target = path_mission.poses.at(cur_point).pose;
            this->SetPosition(pose_target);
            goal = travel;
            cur_point++;

            if (cur_point > path_mission.poses.size()-1){ //
                rev_count++;
                ROS_INFO_STREAM("Circled Building " << rev_count << " times.");

                if(rev_count > scout_rev) {
                    goal = rtl;
                    rev_count = 0;
                    cur_point = 0;
                }
                else{
                    cur_point = 0;
                }
            }
        break;*/
    
    
}

void UAVControl::RunGlobal()
{
    if(goal == travel){
        this->SetMode("AUTO.MISSION");
    }
    else if(goal == hold){
        this->SetMode("AUTO.HOLD");
    }
    else if(goal == rtl){
        this->SetMode("AUTO.RTL");
    }
}

void UAVControl::PauseMission()
{
    mission_mode = paused;
    goal_prev = goal;
    goal = hold;
}

void UAVControl::ResumeMission()
{
    mission_mode = active;
    goal = goal_prev;
}

void UAVControl::StopMission()
{
    mission_mode = stopped;
    goal = rtl;
}

void UAVControl::StopMission(std::string flight_mode)
{
    mission_mode = stopped;
    this->SetMode(flight_mode);
    goal = idle;
}


}//End Namespace
