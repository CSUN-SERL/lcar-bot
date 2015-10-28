#ifndef SIMPLE_CONTROL
#define SIMPLE_CONTROL

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <mavros/mavros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Waypoint.h>
#include <mavros_msgs/WaypointGOTO.h>
#include <geometry_msgs/PoseStamped.h>

#define QUEUE_SIZE 100 //Message Queue size for publishers

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
      Sends the UAV to the desired waypoint. If the UAV is already in air, it
      ascends or descends to the correct altitude and travels to the waypoint.
      Otherwise, the flight mode is changed to Guided and the UAV is armed for
      takeoff. The UAV then goes to the correct altitude and waypoint.

      @param lat Latitude
      @param lon Longitude
      @param alt Altitude
  */
  void GoToWP(double lat, double lon, int alt);

  /**
      Override the RC value of the transmitter.

      @param channel Channel to override (1-8)
      @param value New value of the channel
      @param nh Pointer to the NodeHandle object of the publishing class
  */
  void OverrideRC(int channel, int value);

  /**
      Send a new position command to the UAV

      @param x New x position
      @param y New y position
      @param z New z position
  */
  void SetLocalPosition(int x, int y, int z);

  /**
      Change the UAV's roll, pitch, and yaw values. Requires the UAV to be
      hovering (~50% throttle).
      //NOTE: setpoint_attitude/attitude is currently on supported on the APM
      flight stack. Only the px4 flight stack is supported at the moment.
      //TODO: Untested Function! Test with the px4 flight stack.

      @param roll   New roll value in degrees, relative to the horizontal plane
      @param pitch  New pitch value in degrees, relative to the horizontal plane
      @param yaw    New yaw value in degrees, relative to the horizontal plane
  */
  void SetAttitude(int roll, int pitch, int yaw);

private:
  ros::NodeHandle nh_simple_control;
  ros::ServiceClient sc_arm, sc_takeoff, sc_land, sc_mode, sc_wp_goto;
  ros::Publisher pub_override_rc, pub_setpoint_position, pub_setpoint_attitude;
};

#endif
