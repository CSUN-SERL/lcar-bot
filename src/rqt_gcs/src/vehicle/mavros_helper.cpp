
#include "vehicle/mavros_helper.h"

namespace rqt_gcs
{

MavrosHelper::MavrosHelper(int uav_id) :  //Class constructor
VehicleControl(uav_id)
{
    this->InitialSetup();
}

MavrosHelper::~MavrosHelper()
{
    //Class destructor
}

void MavrosHelper::InitialSetup()
{
    ros::NodeHandle nh(DEF_NS + std::to_string(id));

    //Initialize Service Clients
    sc_arm      = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    sc_takeoff  = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    sc_land     = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    sc_mode     = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    sc_mission  = nh.serviceClient<mavros_msgs::WaypointPush>("mavros/mission/push");

    //Initialize Publisher Objects
    pub_override_rc         = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override",QUEUE_SIZE);
    pub_setpoint_position   = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",QUEUE_SIZE);
    pub_setpoint_gposition  = nh.advertise<mavros_msgs::GlobalPositionTarget>("mavros/setpoint_raw/global",QUEUE_SIZE);
    pub_setpoint_attitude   = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_attitude/attitude",QUEUE_SIZE);
    pub_angular_vel         = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_attitude/cmd_vel",QUEUE_SIZE);
    pub_linear_vel          = nh.advertise<geometry_msgs::TwistStamped>("mavros/setpoint_velocity/cmd_vel",QUEUE_SIZE);
    pub_setpoint_accel      = nh.advertise<geometry_msgs::Vector3Stamped>("mavros/setpoint_accel/accel",QUEUE_SIZE);

    //Initialize Subscribers
    sub_state      = nh.subscribe("mavros/state", QUEUE_SIZE, &MavrosHelper::StateCallback, this);
    sub_battery    = nh.subscribe("mavros/battery", QUEUE_SIZE, &MavrosHelper::BatteryCallback, this);
    sub_imu        = nh.subscribe("mavros/imu/data", QUEUE_SIZE, &MavrosHelper::ImuCallback, this);
    sub_altitude   = nh.subscribe("mavros/global_position/rel_alt", QUEUE_SIZE, &MavrosHelper::RelAltitudeCallback, this);
    sub_heading    = nh.subscribe("mavros/global_position/compass_hdg", QUEUE_SIZE, &MavrosHelper::HeadingCallback, this);
    sub_vel        = nh.subscribe("mavros/local_position/velocity", QUEUE_SIZE, &MavrosHelper::VelocityCallback, this);
    sub_pos_global = nh.subscribe("mavros/global_position/global", QUEUE_SIZE, &MavrosHelper::NavSatFixCallback, this);
    sub_pos_local  = nh.subscribe("mavros/local_position/pose", QUEUE_SIZE, &MavrosHelper::LocalPosCallback, this);
    sub_depth      = nh.subscribe("object_avoidance/depth", QUEUE_SIZE, &MavrosHelper::DepthCallback, this);

    battery.percentage = -1;
}

void MavrosHelper::Arm(bool value)
{
    if(state.armed != value){ //Only change to new state if it's different

        //Create a message for arming/disarming
        mavros_msgs::CommandBool arm;
        arm.request.value = value;

        //Call the service
        if(sc_arm.call(arm)){
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

    //Reset the counter for making service calls
    tries = 0;
}

void MavrosHelper::Takeoff(int altitude)
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

void MavrosHelper::Land()
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

void MavrosHelper::SetMode(std::string mode)
{
    //Create a message for changing flight mode
    mavros_msgs::SetMode new_mode;
    new_mode.request.base_mode = 0;
    new_mode.request.custom_mode = mode; //custom_mode expects a char*

    //Call the service
    if((state.mode).compare(mode) == 0 || !CanRequest()){
        //Already in requested mode or cannot yet request. Do nothing.
    }
    else{
        if(tries < MAX_TRIES){
            if(sc_mode.call(new_mode)){
                if(new_mode.response.success == 1) ROS_INFO_STREAM("Mode changed to " << mode << ".");
                else ROS_ERROR_STREAM("Failed to change flight mode to " << mode << ".");
            }
            else{
                ROS_INFO_STREAM("Mode: " << mode);
                ROS_ERROR_STREAM("Failed to call change flight mode service!");
            }
            last_request = ros::Time::now();

            tries++;
        }
        else{
            ROS_ERROR_ONCE("Failed to change flight mode! Max number of retries reached.");
        }
    }
}

void MavrosHelper::EnableOffboard()
{
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = pose_previous.position.x;
    pose.pose.position.y = pose_previous.position.y;
    pose.pose.position.z = pose_previous.position.z;

    this->Arm(true);

    ros::Rate loop_rate(50); //50Hz
    //send a few setpoints before starting
    for(int i = 10; ros::ok() && i > 0; --i){
        pub_setpoint_position.publish(pose);
        ros::spinOnce();
        loop_rate.sleep();
    }

    this->SetMode("OFFBOARD");
}

sensor_msgs::NavSatFix MavrosHelper::GetLocation()
{
    sensor_msgs::NavSatFix nav;
    nav.latitude = pos_global.latitude;
    nav.longitude = pos_global.longitude;
    return nav;
}

void MavrosHelper::SendMission(mavros_msgs::WaypointPush msg_mission)
{
    //Call the service
    if(sc_mission.call(msg_mission)){
        ROS_INFO_STREAM("Response from service: " << msg_mission.response);
    }
    else{
        ROS_ERROR_STREAM("Failed to call msg_mission service!");
    }
}

void MavrosHelper::OverrideRC(int channel, int value)
{
    //Create the message object
    mavros_msgs::OverrideRCIn override_msg;

    // Update the message with the new RC value
    override_msg.channels[channel-1] = value;

    //Publish the message
    pub_override_rc.publish(override_msg);
}

void MavrosHelper::SetPosition(float x, float y, float z, float yaw)
{
    if(position_mode == global){
        mavros_msgs::GlobalPositionTarget target_global;

        target_global.latitude  = x;
        target_global.longitude = y;
        target_global.altitude  = z;

        this->SetPosition(target_global);
    }
    else{
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
}

void MavrosHelper::SetPosition(geometry_msgs::Pose new_pose)
{
    if(position_mode == global){
        mavros_msgs::GlobalPositionTarget target_global;

        target_global.latitude  = new_pose.position.x;
        target_global.longitude = new_pose.position.y;
        target_global.altitude  = new_pose.position.z;

        this->SetPosition(target_global);
    }
    else{
        //Create the message object
        geometry_msgs::PoseStamped position_stamped;

        //Update the message with the new position
        position_stamped.pose = new_pose;

        //Publish the message
        pub_setpoint_position.publish(position_stamped);
    }
}

void MavrosHelper::SetPosition(mavros_msgs::GlobalPositionTarget new_pose)
{
    //Publish the message
    pub_setpoint_gposition.publish(new_pose);
}

void MavrosHelper::SetAttitude(float roll, float pitch, float yaw)
{
    //Create the message to be published
    geometry_msgs::PoseStamped msg_pose;

    //Construct a Quaternion from Fixed angles and update pose
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    quaternionTFToMsg(q, msg_pose.pose.orientation);

    //Publish the message
    pub_setpoint_attitude.publish(msg_pose);
}

void MavrosHelper::SetAngularVelocity(float roll_vel, float pitch_vel, float yaw_vel)
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

void MavrosHelper::SetLinearVelocity(float x, float y, float z)
{
    geometry_msgs::TwistStamped msg_linear_vel;

    msg_linear_vel.twist.linear.x = x;
    msg_linear_vel.twist.linear.y = y;
    msg_linear_vel.twist.linear.z = z;

    pub_linear_vel.publish(msg_linear_vel);
}

void MavrosHelper::SetAcceleration(float x, float y, float z)
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

bool MavrosHelper::CanRequest()
{
    return ((ros::Time::now() - last_request) > ros::Duration(SC_INTERVAL));
}

}//End Namespace
