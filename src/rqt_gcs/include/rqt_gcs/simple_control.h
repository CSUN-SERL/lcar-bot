#ifndef SIMPLE_CONTROL
#define SIMPLE_CONTROL

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
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
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#define QUEUE_SIZE 100            //Message Queue size for publishers
#define CHECK_FREQUENCY 1         //Frequency for checking change of state
#define TIMEOUT 3*CHECK_FREQUENCY //3 Second timeout
#define TRAVEL_WT 0.5
#define SCOUT_WT 0.5
#define THRESHOLD_XY 1
#define THRESHOLD_Z 0.2
#define ALT_RTL 3
#define BATTERY_MIN 0.30  //Minimum battery level for RTL
#define DEF_NS "UAV"

//Enumerators
enum Mode{
    travel,
    scout,
    rtl,
    land,
    disarm,
    idle
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
  SimpleControl(int uav_id);
  ~SimpleControl();

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

      @param x X coordinate of the local position of the building
      @param y Y coordinate of the local position of the building
      @param z The height at which the UAV should arrive at the building
  */
  void ScoutBuilding(float x, float y, float z);

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

      @param x New x position
      @param y New y position
      @param z New z position
  */
  void SetLocalPosition(int x, int y, int z);

  /**
      Send a new position command to the UAV.

      @param new_pose The new local position passed as a Pose object
  */
  void SetLocalPosition(geometry_msgs::Point new_point);

  /**
      Change the UAV's roll, pitch, and yaw values. Requires the UAV to be
      hovering (~50% throttle).
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.

      @param roll   New roll value in degrees, relative to the horizontal plane
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
  void SetAngularVelocity(int roll_vel, int pitch_vel, int yaw_vel);

  /**
      Change the UAV's linear velocity for roll, pitch, and yaw.

      @param x   New roll velocity
      @param y  New pitch velocity
      @param z    New yaw velocity
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
      Compare two geometry_msgs::Point objects within a threshold value.

      @param point1  First point to compare
      @param point2  Second point to compare
  */
  int ComparePosition(geometry_msgs::Point point1, geometry_msgs::Point point2);

  /**
      Calculate the distance between two points.

      @param point1  First point
      @param point2  Second point
  */
  int CalculateDistance(geometry_msgs::Point point1, geometry_msgs::Point point2);

  /**
      Calculate a Vector3d object that defines the displacement for reaching a
      point on a circle.

      @param angle  Angle, in degrees, for which the next Vector should be
                    generated.
  */
  Eigen::Vector3d CircleShape(int angle);

  /**
      Manage the UAV and ensure that it completes the mission

      @param index The current point number the quad is traveling to.
  */
  geometry_msgs::Point DiamondShape(int index);


  void Run();


  void SetRTL() { goal = rtl; }

  //Getter Functions
  mavros_msgs::State GetState() { return state; }
  mavros_msgs::BatteryStatus GetBatteryStatus() { return battery; }
  sensor_msgs::Imu  GetImu() { return imu; }
  FlightState GetFlightState() { return UpdateFlightState(); }
  int GetDistanceToWP() { return CalculateDistance(pos_target, pos_local); }
  float GetMissionProgress();

private:

  //Callback Prototypes
  void StateCallback(const mavros_msgs::State& msg_state) { state = msg_state; }
  void BatteryCallback(const mavros_msgs::BatteryStatus& msg_battery) { battery = msg_battery; }
  void ImuCallback(const sensor_msgs::Imu& msg_imu) { imu = msg_imu; }
  void RelAltitudeCallback(const std_msgs::Float64& msg_altitude) { altitude_rel = msg_altitude.data; }
  void HeadingCallback(const std_msgs::Float64& msg_heading) { heading_deg = msg_heading.data; }
  void VelocityCallback(const geometry_msgs::TwistStamped& msg_vel) { velocity = msg_vel; }
  void NavSatFixCallback(const sensor_msgs::NavSatFix& msg_gps) { pos_global = msg_gps; }
  void LocalPosCallback(const geometry_msgs::PoseStamped& msg_pos) { pos_local = msg_pos.pose.position; }

  //For returning Flight State Data to GCS
  FlightState UpdateFlightState();

  //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
  ros::NodeHandle     nh_simple_control;
  ros::ServiceClient  sc_arm,
                      sc_takeoff,
                      sc_land,
                      sc_mode,
                      sc_mission;
  ros::Publisher      pub_override_rc,
                      pub_setpoint_position,
                      pub_setpoint_attitude,
                      pub_angular_vel,
                      pub_linear_vel,
                      pub_setpoint_accel;
  ros::Subscriber     sub_state,
                      sub_battery,
                      sub_imu,
                      sub_pos_global,
                      sub_pos_local,
                      sub_altitude,
                      sub_heading,
                      sub_vel;

  //UAV State Variables
  std::string ns;
  mavros_msgs::State state;
  mavros_msgs::BatteryStatus battery;
  sensor_msgs::Imu imu;
  sensor_msgs::NavSatFix pos_global;
  geometry_msgs::TwistStamped velocity;
  geometry_msgs::Point  pos_local,
                        pos_target,
                        pos_home,
                        pos_previous;
  float altitude_rel, heading_deg;
  //int goal = IDLE;
  Mode goal = idle;
  ros::Time last_request;
};

#endif
