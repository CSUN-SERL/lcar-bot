#ifndef SIMPLE_CONTROL
#define SIMPLE_CONTROL

#include <stdio.h>
#include <vector>
#include <tf/tf.h>
#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <termios.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/BatteryStatus.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <lcar_msgs/Door.h>
#include <lcar_msgs/Target.h>

#include <rqt_gcs/access_point.h>

#define PI 3.14159265
#define QUEUE_SIZE 100            //Message Queue size for publishers
#define CHECK_FREQUENCY 1         //Frequency for checking change of state
#define TIMEOUT 3*CHECK_FREQUENCY //3 Second timeout
#define TRAVEL_WT 0.5
#define SCOUT_WT 0.5
#define THRESHOLD_XY 0.08
#define THRESHOLD_Z 0.08
#define THRESHOLD_YAW 0.1
#define THRESHOLD_DEPTH 2
#define ALT_RTL 2
#define BATTERY_MIN 0.10  //Minimum battery level for RTL
#define DEF_NS "UAV"

//Enumerators
enum Mode{
    travel,
    hold_pose,
    scout,
    rtl,
    land,
    disarm,
    idle,
    null
};

//Structs
struct FlightState {
    float roll, pitch, yaw;
    float altitude;
    float vertical_speed, ground_speed;
    float heading;
};

class SimpleControl
{
public:
    int id;
    static int static_id;
    SimpleControl();
    SimpleControl(int uav_id);
    ~SimpleControl();
    
    int accepted_images = 0,
        rejected_images = 0;
    
    /**
     * set online mode on or off
     * 
     * @param value: pass true for online, false for offline
     */
    bool setOnlineMode(bool value)
    {
       online_mode = value;
    }
    /**
      Arm or disarm the UAV.

      @param value Pass true for arm, false for disarm
    */
    void Arm(bool value);

    /**
      Takeoff to a set altitude. Requires the UAV to be first armed and then
      put into Guided mode.

      @param altitude Altitude, in feet, for takeoff
    */
    void Takeoff(int altitude);

    /**
      Land the UAV
    */
    void Land();

    /**
      Set the UAV Flight Mode.

      @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
      Loiter, RTL, or Circle
    */
    void SetMode(std::string mode);

    /**
      Enable OFFBOARD mode on the PX4
    */
    void EnableOffboard();

    /**
      Returns the current location of the UAV in JSON format.
    */
    std::string GetLocation();

    /**
      Add the passed GPS location to the current set of waypoints to visit.

      @param lat Latitude
      @param lon Longitude
      @param alt Altitude
    */
    void SetWayPoint(double lat, double lon, int alt);
    
    /**
      Overloaded function for SetWayPoint(double lat, double lon, int alt) that
      accepts a string parameter as the coordinate.

      @param waypoint A String containing the GPS coordinates of the WayPoint
    */
    void SetWayPoint(std::string waypoint);

    /**
      Executes proper instructions for running the Scout Building play

      @param target_point query_msgs::Target building location

    */
    void ScoutBuilding(lcar_msgs::Target msg_target);

    /**
      Send a list of waypoints (mission) to the UAV.
      @param mission_file Name of the text file that contains the mision
    */
    void SendMission(std::string mission_file);

    /**
      Override the RC value of the transmitter.

      @param channel Channel to override (1-8)
      @param value New value of the channel
    */
    void OverrideRC(int channel, int value);

    /**
      Send a new position command to the UAV.

      @param x      New x position
      @param y      New y position
      @param z      New z position
      @param yaw    New yaw value in degrees
    */
    void SetLocalPosition(float x, float y, float z, float yaw = 361);

    /**
      Send a new position command to the UAV.

      @param new_pose The new local position passed as a Pose object
    */
    void SetLocalPosition(geometry_msgs::Pose new_pose);

    /**
      Change the UAV's roll, pitch, and yaw values. Requires the UAV to be
      hovering (~50% throttle).
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.

      @param roll   New roll value in degrees, relastd::tive to the horizontal plane
      @param pitch  New pitch value in degrees, relative to the horizontal plane
      @param yaw    New yaw value in degrees, relative to the horizontal plane
    */
    void SetAttitude(float roll, float pitch, float yaw);

    /**
      Change the UAV's angular velocity for roll, pitch, and yaw.
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param roll_vel   New roll velocity
      @param pitch_vel  New pitch velocity
      @param yaw_vel    New yaw velocity
    */
    void SetAngularVelocity(float roll_vel, float pitch_vel, float yaw_vel);

    /**
      Change the UAV's linear velocity for roll, pitch, and yaw.

      @param x  New roll velocity
      @param y  New pitch velocity
      @param z  New yaw velocity
    */
    void SetLinearVelocity(float x, float y, float z);

    /**
      Change the UAV's acceleration for roll, pitch, and yaw.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param x  Acceleration X
      @param y  Acceleration Y
      @param z  Acceleration Z
    */
    void SetAcceleration(float x, float y, float z);

    /**
      Manage the UAV and ensure that it is stable
    */
    void Run();

    /**
      Return to launch site
    */
    void SetRTL() { this->EnableOffboard(); goal = rtl; }

    /**
      Manage the UAV and ensure that it is stable
    */
    void SendDoorResponse(lcar_msgs::Door msg_answer) { pub_door_answer.publish(msg_answer); }

    
    //Getter Functions
    mavros_msgs::State GetState() { return state; }
    mavros_msgs::BatteryStatus GetBatteryStatus() { return battery; }
    sensor_msgs::Imu  GetImu() { return imu; }
    FlightState GetFlightState() { return UpdateFlightState(); }
    int GetDistanceToWP() { return CalculateDistance(pose_target, pose_local); }
    float GetMissionProgress();
    std::vector<AccessPoint>* GetRefAccessPoints() { return &access_pts; }
    std::vector<lcar_msgs::DoorPtr>* GetDoorQueries() { return &queries_door; }
    bool GetContactStatus() { return recieved_heartbeat; }

private:
    void InitialSetup();

    //Private Class Functions
    /**
      Compare two geometry_msgs::Point objects within a threshold value.

      @param point1  First point to compare
      @param point2  Second point to compare
    */
    int ComparePosition(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    /**
      Calculate the distance between two points.

      @param point1  First point
      @param point2  Second point
    */
    int CalculateDistance(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

    /**
      Calculate a Vector3d object that defines the displacement for reaching a
      point on a circle.

      @param target_point   Center point for target building path generation
    */
    nav_msgs::Path CircleShape(lcar_msgs::Target target_point);

    /**
      Manage the UAV and ensure that it completes the mission

      @param index The current point number the quad is traveling to.
    */
    nav_msgs::Path DiamondShape(lcar_msgs::Target target_point);

    //Callback Prototypes
    void StateCallback(const mavros_msgs::State& msg_state) { state = msg_state; }
    void BatteryCallback(const mavros_msgs::BatteryStatus& msg_battery) { battery = msg_battery; }
    void ImuCallback(const sensor_msgs::Imu& msg_imu) { imu = msg_imu; }
    void RelAltitudeCallback(const std_msgs::Float64& msg_altitude) { altitude_rel = msg_altitude; }
    void HeadingCallback(const std_msgs::Float64& msg_heading) { heading_deg = msg_heading; }
    void VelocityCallback(const geometry_msgs::TwistStamped& msg_vel) { velocity = msg_vel; }
    void NavSatFixCallback(const sensor_msgs::NavSatFix& msg_gps) { pos_global = msg_gps; }
    void LocalPosCallback(const geometry_msgs::PoseStamped& msg_pos) { pose_local = msg_pos.pose; }
    void DepthCallback(const std_msgs::Float64& msg_depth){ object_distance = msg_depth;}
    //void DoorQueryCallback(const lcar_msgs::Door& msg_query){ queries_door.push_back(msg_query); }
    void DetectionCallback(const lcar_msgs::DoorPtr& msg_detection)
    {
        AccessPoint new_point;

        new_point.SetTime(ros::Time::now());
        new_point.SetImage((sensor_msgs::Image)msg_detection->framed_picture);
        new_point.SetAltitude(altitude_rel);
        new_point.SetHeading(heading_deg);
        new_point.SetLocation(pos_global);
        new_point.SetType(AccessPoint::door);
        access_pts.push_back(new_point);
                
        if(online_mode && msg_detection->query)
            queries_door.push_back(msg_detection);
    }
    void UavHeartbeatCallback(std_msgs::Int32 heartbeat_msg)
    {
        if(connection_sim)
            return;
        
        uav_heartbeat_timer.stop();
        
        recieved_heartbeat = true;
        ROS_INFO_STREAM("received heartbeat for uav_" << id);
        //TODO
            //logic for receiving a heartbeat from the uav
        
        uav_heartbeat_timer.start();
    }
    void UavHeartbeatTimeoutCallback(const ros::TimerEvent& e)
    {
        ROS_ERROR_STREAM("did not recieve heartbeat for uav_" << id);

        recieved_heartbeat = false;

        //TODO
            //gray out vehicle select button on main gui
            //some kind of notification to the user
        //UPDATE
            // this will be handled by SimpleGCS, by querying each uav for the 
            // value of in_contact. if false, gray out its button
        
    }
    void GcsHeartbeatTimeoutCallback(const ros::TimerEvent& e)
    {
        heartbeat.data++;
        pub_heartbeat.publish(heartbeat);
    }

    //For returning Flight State Data to GCS
    FlightState UpdateFlightState();

    //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
    ros::NodeHandle                 nh;
    ros::ServiceClient              sc_arm,
                                    sc_takeoff,
                                    sc_land,
                                    sc_mode,
                                    sc_mission;
    ros::Publisher                  pub_override_rc,
                                    pub_setpoint_position,
                                    pub_setpoint_attitude,
                                    pub_angular_vel,
                                    pub_linear_vel,
                                    pub_setpoint_accel,
                                    pub_door_answer,
                                    pub_heartbeat;
    ros::Subscriber                 sub_state,
                                    sub_battery,
                                    sub_imu,
                                    sub_pos_global,
                                    sub_pos_local,
                                    sub_altitude,
                                    sub_heading,
                                    sub_vel,
                                    sub_vrpn,
                                    sub_depth,
                                    sub_door_query,
                                    sub_detection,
                                    sub_heartbeat;

    //UAV State Variables
    std::string                     ns;
    mavros_msgs::State              state;
    mavros_msgs::BatteryStatus      battery;
    sensor_msgs::Imu                imu;
    sensor_msgs::NavSatFix          pos_global;
    geometry_msgs::TwistStamped     velocity;
    geometry_msgs::Pose             pose_local,
                                    pose_target,
                                    pose_home,
                                    pose_previous;
    nav_msgs::Path                  path_mission;
    std_msgs::Float64               altitude_rel,
                                    heading_deg,
                                    object_distance;
    Mode                            goal = idle,
                                    goal_prev = null;
    ros::Time                       last_request;
    std::vector<AccessPoint>        access_pts;
    std::vector<lcar_msgs::DoorPtr> queries_door;
    bool                            collision = false,
                                    online_mode = true,
                                    connection_sim = false,
                                    recieved_heartbeat = true;
    ros::Timer                      uav_heartbeat_timer,
                                    gcs_heartbeat_timer;
    std_msgs::Int32                 heartbeat;
};

#endif
