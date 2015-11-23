#ifndef SIMPLE_CONTROL
#define SIMPLE_CONTROL

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
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

#define QUEUE_SIZE 100            //Message Queue size for publishers
#define CHECK_FREQUENCY 1         //Frequency for checking change of state
#define TIMEOUT 3*CHECK_FREQUENCY //3 Second timeout

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
      Send a list of waypoints (mission) to the UAV.

      @param mission_file Name of the text file that contains the mision
  */
  void SendMission(std::string mission_file);

  /**
      Start the stored mission on the UAV.

  */
  void BeginMission();

  /**
      Override the RC value of the transmitter.

      @param channel Channel to override (1-8)
      @param value New value of the channel
      @param nh Pointer to the NodeHandle object of the publishing class
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
      Change the UAV's roll, pitch, and yaw values. Requires the UAV to be
      hovering (~50% throttle).
      //NOTE: setpoint_attitude/attitude is currently not supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param roll   New roll value in degrees, relative to the horizontal plane
      @param pitch  New pitch value in degrees, relative to the horizontal plane
      @param yaw    New yaw value in degrees, relative to the horizontal plane
  */
  void SetAttitude(int roll, int pitch, int yaw);

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

  //Getter Functions
  mavros_msgs::State GetState() { return state; }
  mavros_msgs::BatteryStatus GetBatteryStatus() { return battery; }
  sensor_msgs::Imu  GetImu() { return imu; }
  FlightState GetFlightState() { return UpdateFlightState(); }

private:

  //Callback Prototypes
  void StateCallback(const mavros_msgs::State& msg_state) { state = msg_state; }
  void BatteryCallback(const mavros_msgs::BatteryStatus& msg_battery) { battery = msg_battery; }
  void ImuCallback(const sensor_msgs::Imu& msg_imu) { imu = msg_imu; }
  void RelAltitudeCallback(const std_msgs::Float64& msg_altitude) { altitude_rel = msg_altitude.data; }
  void HeadingCallback(const std_msgs::Float64& msg_heading) { heading_deg = msg_heading.data; }
  void VelocityCallback(const geometry_msgs::TwistStamped& msg_vel) { velocity = msg_vel; }
  void NavSatFixCallback(const sensor_msgs::NavSatFix& msg_gps) { pos_global = msg_gps; }

  FlightState UpdateFlightState();

  //ROS NodeHandle, Service Client, Publisher, and Subscriber Variables
  ros::NodeHandle     nh_simple_control;
  ros::ServiceClient  sc_arm, sc_takeoff, sc_land, sc_mode, sc_mission;
  ros::Publisher      pub_override_rc, pub_setpoint_position, pub_setpoint_attitude, pub_angular_vel;
  ros::Subscriber     sub_state, sub_battery, sub_imu, sub_pos_global, sub_pos_local, sub_altitude, sub_heading, sub_vel;

  //UAV State Variables
  mavros_msgs::State state;
  mavros_msgs::BatteryStatus battery;
  sensor_msgs::Imu imu;
  sensor_msgs::NavSatFix pos_global;
  float altitude_rel, heading_deg;
  geometry_msgs::TwistStamped velocity;
  geometry_msgs::PoseWithCovarianceStamped pos_local;
};

#endif
