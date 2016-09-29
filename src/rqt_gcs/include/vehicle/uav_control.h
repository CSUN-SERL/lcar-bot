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
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/WaypointPush.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include "lcar_msgs/Door.h"
#include "lcar_msgs/TargetLocal.h"
#include "lcar_msgs/TargetGlobal.h"

#include "rqt_gcs/access_point.h"
#include "vehicle/vehicle_control.h"

namespace rqt_gcs
{

#define PI 3.14159265
#define QUEUE_SIZE 100
#define PI 3.14159265          //Message Queue size for publishers
#define CHECK_FREQUENCY 1         //Frequency for checking change of state
#define TIMEOUT 3*CHECK_FREQUENCY //3 Second timeout
#define TRAVEL_WT 0.5
#define SCOUT_WT 0.5
#define THRESHOLD_XY 0.08
#define THRESHOLD_Z 0.08
#define THRESHOLD_XY_GPS 0.00001
#define THRESHOLD_Z_GPS 0.5
#define THRESHOLD_YAW 0.1
#define THRESHOLD_GPS 0.001        //Lat & Lon tolerances
#define THRESHOLD_ALT 1            //Altitude tolerance for GPS
#define THRESHOLD_DEPTH 2
#define ALT_RTL 2
#define BATTERY_MIN 0.10    //Minimum battery level for RTL
#define DEF_NS "UAV"
#define R_EARTH 6371        //Earth's radius in km
#define SC_INTERVAL 3       //Time, in seconds, between service calls


class UAVControl : public VehicleControl
{
public:

    int accepted_images = 0,
        rejected_images = 0;

    UAVControl(int uav_id);
    ~UAVControl();

    /**
     * \brief Set online mode on or off
     *
     * @param value: pass true for online, false for offline
     */
    bool SetOnlineMode(bool value)
    {
       online_mode = value;
    }

    /**
    * \brief Arm or disarm the UAV.
    *
    * @param value Pass true for arm, false for disarm
    */
    void Arm(bool value) override;

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
    void SetMode(std::string mode) override;

    /**
      Enable OFFBOARD mode on the PX4
    */
    void EnableOffboard();

    /**
      Returns the current location of the UAV in JSON format.
    */
    sensor_msgs::NavSatFix GetLocation() override;

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
      Executes a Scout Building play using local coordinates

      @param target_point lcar_msgs::TargetLocal building location

    */
    void ScoutBuilding(lcar_msgs::TargetLocal msg_target);

    /**
      Executes a Scout Building play using global coordinates

      @param target_point lcar_msgs::TargetGlobal building location

    */
    void ScoutBuilding(lcar_msgs::TargetGlobal msg_target);

    /**
      Send a list of waypoints (mission) to the UAV.

      @param mission List of waypoints contained in a mission object.
    */
    void SendMission(mavros_msgs::WaypointPush mission);

    /**
      Override the RC value of the transmitter.

      @param channel Channel to override (1-8)
      @param value New value of the channel
    */
    void OverrideRC(int channel, int value);

    /**
      Send a new local position command to the UAV.

      @param x      New x position
      @param y      New y position
      @param z      New z position
      @param yaw    New yaw value in degrees
    */
    void SetPosition(float x, float y, float z, float yaw = 361);

    /**
      Send a new local position command to the UAV.

      @param new_pose The new local position passed as a Pose object
    */
    void SetPosition(geometry_msgs::Pose new_pose);

    /**
      Send a new global position command to the UAV.

      @param new_pose The new global position passed as a GlobalPositionTarget obect
    */
    void SetPosition(mavros_msgs::GlobalPositionTarget new_pose);

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
    void SetRTL() override { this->EnableOffboard(); goal = Mode::rtl; }

    /**
      Manage the UAV and ensure that it is stable
    */
    void SendDoorResponse(lcar_msgs::Door msg_answer) { pub_door_answer.publish(msg_answer); }

    /*!
     * \brief Manages heartbeat emission
     * \param drop_connection True or false
     *
     * Simulates a drop in connection to the ground control station and stops
     * emitting heartbeats.
     */
    void DropConnection(bool drop_connection)
    {
        connection_dropped = drop_connection;
    }

    /*!
     * \brief Pause the current mission
     *
     * Stops the UAV at its current location and waits for a command from the
     * GCS to resume the mission.
     */
    void PauseMission() override;

    /*!
     * \brief Resumes the previously paused mission.
     *
     * Starts the mission that was previously paused. If there was no previously
     * paused mission, does nothing.
     */
    void ResumeMission() override;

    /*!
     * \brief Cancels the current mission
     *
     * Cancels the current mission and commands the UAV to return base.
     */
    void StopMission();

    /*!
     * \brief Cancels the current mission
     * \param flight_mode The flight mode to switch to after cancelling mission
     *
     * Cancels the current mission and changes flight mode to the one selected
     * by the user.
     */
    void StopMission(std::string flight_mode);

    //Getter Functions
    mavros_msgs::State GetState() { return state; }
    sensor_msgs::BatteryState GetBatteryState() { return battery; }
    sensor_msgs::Imu  GetImu() { return imu; }
    FlightState GetFlightState() { return UpdateFlightState(); }
    int GetDistanceToWP() { return CalculateDistance(pose_target, pose_local); }
    float GetMissionProgress();
    MissionMode GetMissionMode(){return mission_mode;}
    std::vector<AccessPoint>* GetRefAccessPoints() { return &access_pts; }
    std::vector<lcar_msgs::DoorPtr>* GetDoorQueries() { return &queries_door; }
    bool RecievedHeartbeat() { return heartbeat_recieved; }
    Mode getMode(){ return goal; }

    void SetRejected_images(int rejected_images)
    {
        this->rejected_images = rejected_images;
    }

    int GetRejected_images() const
    {
        return rejected_images;
    }

    void SetAccepted_images(int accepted_images)
    {
        this->accepted_images = accepted_images;
    }

    int GetAccepted_images() const
    {
        return accepted_images;
    }

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
      Compare two geometry_msgs::Point objects within a threshold value. Only looks at altitude.

      @param point1  First point to compare
      @param point2  Second point to compare
    */
    int CompareAltitude(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2);

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
    nav_msgs::Path CircleShape(lcar_msgs::TargetLocal target_point);

    /**
      Calculate a list of waypoints in a circular shape around the target GPS location

      @param target_point   Center point of the circle
      @return Waypoint Mission item.
    */
    mavros_msgs::WaypointPush CircleShape(lcar_msgs::TargetGlobal target_point);

    /**
      Check critical sensor values
    */
    void SafetyCheck();

    /**
      Manage a local mission
    */
    void RunLocal();

    /**
      Manage a global mission
    */
    void RunGlobal();

    /*!
     * \brief Determines if it is safe to call a service
     * \return True if safe, false otherwise
     */
    bool CanRequest();

    //Callback Prototypes
    void StateCallback(const mavros_msgs::State& msg_state) { state = msg_state; }
    void BatteryCallback(const sensor_msgs::BatteryState& msg_battery) { battery = msg_battery; }
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
        if(connection_dropped)
            return;

        timer_heartbeat_uav.stop();

        heartbeat_recieved = true;
        uav_heartbeat.data = heartbeat_msg.data;

        timer_heartbeat_uav.start();
    }

    void UavHeartbeatTimeoutCallback(const ros::TimerEvent& e)
    {
        heartbeat_recieved = false;
    }

    void GcsPublishHeartbeat(const ros::TimerEvent& e)
    {
        gcs_heartbeat.data++;
        pub_heartbeat.publish(gcs_heartbeat);
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
                                    pub_setpoint_gposition,
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
    sensor_msgs::BatteryState       battery;
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
    Mode                      goal = idle,
                                    goal_prev = null;
    PositionMode                    position_mode = local;
    MissionMode                     mission_mode = stopped;
    ros::Time                       last_request;
    std::vector<AccessPoint>        access_pts;
    std::vector<lcar_msgs::DoorPtr> queries_door;
    bool                            collision = false,
                                    online_mode = true,
                                    connection_dropped = false,
                                    heartbeat_recieved = true;
    ros::Timer                      timer_heartbeat_uav,
                                    timer_heartbeat_gcs;
    std_msgs::Int32                 gcs_heartbeat,
                                    uav_heartbeat;
};

}
#endif
