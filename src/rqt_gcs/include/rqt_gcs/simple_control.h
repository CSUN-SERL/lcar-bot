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

#include <mavros/mavros.h>
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
#define TRAVEL 0
#define SCOUT 1
#define RTL 2
#define LAND 3
#define DISARM 4
#define TRAVEL_WT 0.5
#define SCOUT_WT 0.5
#define THRESHOLD_XY 1
#define THRESHOLD_Z 1
#define ALT_RTL 3
#define BATTERY_MIN 0.30  //Minimum battery level for RTL
#define NUM_UAV 2         //Total number of UAV's in the system

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
  SimpleControl();
  ~SimpleControl();

  /**
      Arm or disarm the UAV.

      @param value Pass true for arm, false for disarm
      @param uav_num The target UAV number
  */
  void Arm(bool value, int uav_num);

  /**
      Takeoff to a set altitude. Requires the UAV to be first armed and then
      put into Guided mode.

      @param altitude Altitude, in feet, for takeoff
      @param uav_num The target UAV number
  */
  void Takeoff(int altitude, int uav_num);

  /**
      Land the UAV

      @param uav_num The target UAV number
  */
  void Land(int uav_num);

  /**
      Set the UAV Flight Mode.

      @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
      Loiter, RTL, or Circle
      @param uav_num The target UAV number
  */
  void SetMode(std::string mode, int uav_num);

  /**
      Returns the current location of the UAV in JSON format.
      @param uav_num The target UAV number
  */
  std::string GetLocation(int uav_num);

  /**
      Add the passed GPS location to the current set of waypoints to visit.

      @param lat Latitude
      @param lon Longitude
      @param alt Altitude
      @param uav_num The target UAV number
  */
  void SetWayPoint(double lat, double lon, int alt, int uav_num);

  /**
      Overloaded function for SetWayPoint(double lat, double lon, int alt) that
      accepts a string parameter as the coordinate.

      @param waypoint A String containing the GPS coordinates of the WayPoint
  */
  void SetWayPoint(std::string waypoint, int uav_num);

  /**
      Executes proper instructions for running the Scout Building play

      @param x X coordinate of the local position of the building
      @param y Y coordinate of the local position of the building
      @param z The height at which the UAV should arrive at the building
      @param uav_num The target UAV number
  */
  void ScoutBuilding(int x, int y, int z, int uav_num);

  /**
      Override the RC value of the transmitter.

      @param channel Channel to override (1-8)
      @param value New value of the channel
      @param uav_num The target UAV number
  */
  void OverrideRC(int channel, int value, int uav_num);

  /**
      Send a new position command to the UAV.

      @param x New x position
      @param y New y position
      @param z New z position
  */
  void SetLocalPosition(int x, int y, int z, int  uav_num);

  /**
      Send a new position command to the UAV.

      @param new_pose The new local position passed as a Pose object
      @param uav_num The target UAV number
  */
  void SetLocalPosition(geometry_msgs::Point new_point, int uav_num);

  /**
      Change the UAV's roll, pitch, and yaw values. Requires the UAV to be
      hovering (~50% throttle).
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.

      @param roll   New roll value in degrees, relative to the horizontal plane
      @param pitch  New pitch value in degrees, relative to the horizontal plane
      @param yaw    New yaw value in degrees, relative to the horizontal plane
      @param uav_num The target UAV number
  */
  void SetAttitude(float roll, float pitch, float yaw, int uav_num);

  /**
      Change the UAV's angular velocity for roll, pitch, and yaw.
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param roll_vel   New roll velocity
      @param pitch_vel  New pitch velocity
      @param yaw_vel    New yaw velocity
      @param uav_num The target UAV number
  */
  void SetAngularVelocity(int roll_vel, int pitch_vel, int yaw_vel, int uav_num);

  /**
      Change the UAV's linear velocity for roll, pitch, and yaw.

      @param x  New roll velocity
      @param y  New pitch velocity
      @param z  New yaw velocity
      @param uav_num The target UAV number
  */
  void SetLinearVelocity(float x, float y, float z, int uav_num);

  /**
      Change the UAV's acceleration for roll, pitch, and yaw.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param x  Acceleration X
      @param y  Acceleration Y
      @param z  Acceleration Z
      @param uav_num The target UAV number
  */
  void SetAcceleration(float x, float y, float z, int uav_num);

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

      @param angle    Angle, in degrees, for which the next Vector should be
                      generated.
      @param uav_num  The target UAV number
  */
  Eigen::Vector3d CircleShape(int angle, int uav_num);

  /**
      Manage the UAV and ensure that it completes the mission

      @param uav_num The target UAV number
  */
  void Run(int uav_num);

  void SetRTL() { goal = RTL; }

  //Getter Functions
  mavros_msgs::State GetState(int uav_num) { return state[uav_num-1]; }
  mavros_msgs::BatteryStatus GetBatteryStatus(int uav_num) { return battery[uav_num-1]; }
  sensor_msgs::Imu  GetImu(int uav_num) { return imu[uav_num-1]; }
  FlightState GetFlightState(int uav_num) { return UpdateFlightState(uav_num); }
  //TODO:Fix this function
  int GetDistanceToWP(int uav_num) { return CalculateDistance(pos_target[uav_num-1], pos_local[uav_num-1]); }
  float GetMissionProgress(int uav_num);

private:

  //Callback Functions
  void StateCallback(const ros::MessageEvent<mavros_msgs::State>& event_state)
  {
    int index = TopicToIndex(event_state.getPublisherName());
    state[index] = *(event_state.getMessage()); //Dereference before equating
  }
  void BatteryCallback(const ros::MessageEvent<mavros_msgs::BatteryStatus>& event_battery)
  {
    int index = TopicToIndex(event_battery.getPublisherName());
    battery[index] = *(event_battery.getMessage());
  }
  void ImuCallback(const ros::MessageEvent<sensor_msgs::Imu>& event_imu)
  {
    int index = TopicToIndex(event_imu.getPublisherName());
    imu[index] = *(event_imu.getMessage());
  }
  void RelAltitudeCallback(const ros::MessageEvent<std_msgs::Float64>& event_altitude)
  {
    int index = TopicToIndex(event_altitude.getPublisherName());
    altitude_rel[index] = (*(event_altitude.getMessage())).data;
  }
  void HeadingCallback(const ros::MessageEvent<std_msgs::Float64>& event_heading)
  {
    int index = TopicToIndex(event_heading.getPublisherName());
    heading_deg[index] = (*(event_heading.getMessage())).data;
  }
  void VelocityCallback(const ros::MessageEvent<geometry_msgs::TwistStamped>& event_vel)
  {
    int index = TopicToIndex(event_vel.getPublisherName());
    velocity[index] = *(event_vel.getMessage());
  }
  void NavSatFixCallback(const ros::MessageEvent<sensor_msgs::NavSatFix>& event_gps)
  {
    int index = TopicToIndex(event_gps.getPublisherName());
    pos_global[index] = *(event_gps.getMessage());
  }
  void LocalPosCallback(const ros::MessageEvent<geometry_msgs::PoseStamped>& event_pos)
  {
    int index = TopicToIndex(event_pos.getPublisherName());
    pos_local[index] = (*(event_pos.getMessage())).pose.position;
  }

  int TopicToIndex(std::string topic_name)
  {
    std::string str_index = topic_name.substr(4,1);
    return atoi(str_index.c_str())-1;
  }

  //For returning Flight State Data to GCS
  FlightState UpdateFlightState(int uav_num);

  //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
  ros::NodeHandle     nh_simple_control;
  ros::ServiceClient  sc_arm[NUM_UAV],
                      sc_takeoff[NUM_UAV],
                      sc_land[NUM_UAV],
                      sc_mode[NUM_UAV],
                      sc_mission[NUM_UAV];
  ros::Publisher      pub_override_rc[NUM_UAV],
                      pub_setpoint_position[NUM_UAV],
                      pub_setpoint_attitude[NUM_UAV],
                      pub_angular_vel[NUM_UAV],
                      pub_linear_vel[NUM_UAV],
                      pub_setpoint_accel[NUM_UAV];
  ros::Subscriber     sub_state[NUM_UAV],
                      sub_battery[NUM_UAV],
                      sub_imu[NUM_UAV],
                      sub_pos_global[NUM_UAV],
                      sub_pos_local[NUM_UAV],
                      sub_altitude[NUM_UAV],
                      sub_heading[NUM_UAV],
                      sub_vel[NUM_UAV];

  //UAV State Variables
  std::string uav_ns = "UAV"; //Default namespace
  mavros_msgs::State state[NUM_UAV];
  mavros_msgs::BatteryStatus battery[NUM_UAV];
  sensor_msgs::Imu imu[NUM_UAV];
  sensor_msgs::NavSatFix pos_global[NUM_UAV];
  geometry_msgs::TwistStamped velocity[NUM_UAV];
  geometry_msgs::Point  pos_local[NUM_UAV],
                        pos_target[NUM_UAV],
                        pos_home[NUM_UAV],
                        pos_previous[NUM_UAV];
  float altitude_rel[NUM_UAV], heading_deg[NUM_UAV];
  int goal = -1;
};

#endif
