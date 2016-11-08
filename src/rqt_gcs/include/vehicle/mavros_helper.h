#ifndef MAVROS_HELPER
#define MAVROS_HELPER

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
#include <util/data_types.h>

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
#include <vehicle/vehicle_control.h>

#define PI 3.14159265
#define QUEUE_SIZE 100
#define CHECK_FREQUENCY 1         //Frequency for checking change of state
#define TIMEOUT 3*CHECK_FREQUENCY //3 Second timeout
#define SC_INTERVAL 3       //Time, in seconds, between service calls
#define MAX_TRIES 5


namespace rqt_gcs 
{

class MavrosHelper: public VehicleControl
{
public:

    MavrosHelper(int id);
    ~MavrosHelper();

    /**
    * \brief Arm or disarm the UAV.
    *
    * @param value Pass true for arm, false for disarm
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
    sensor_msgs::NavSatFix GetLocation();

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

    //Getter Functions
    mavros_msgs::State GetState()               { return state; }
    sensor_msgs::BatteryState GetBatteryState() { return battery; }
    sensor_msgs::Imu  GetImu()                  { return imu; }

protected:

    //UAV State Variables
    mavros_msgs::State              state;
    sensor_msgs::BatteryState       battery;
    sensor_msgs::Imu                imu;
    sensor_msgs::NavSatFix          pos_global;
    geometry_msgs::TwistStamped     velocity;
    geometry_msgs::Pose             pose_local;
    std_msgs::Float64               altitude_rel,
                                    heading_deg;
    ros::Time                       last_request;
    PositionMode                    position_mode = local;
    int                             tries = 0;

private:
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

    //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
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
                                    pub_setpoint_accel;

    ros::Subscriber                 sub_state,
                                    sub_battery,
                                    sub_imu,
                                    sub_pos_global,
                                    sub_pos_local,
                                    sub_altitude,
                                    sub_heading,
                                    sub_vel;

};

}//End Namespace

#endif
