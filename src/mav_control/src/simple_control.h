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
#include <geometry_msgs/PoseStamped.h>

#define QUEUE_SIZE 1000 //Message Queue size for publishers

class SimpleControl
{
public:
  SimpleControl();
  ~SimpleControl();

  /**
      Arm or disarm the UAV

      @param value Pass true for arm, false for disarm
  */
  void Arm(bool value);

  /**
      Land the UAV
  */
  void Land();

  /**
      Set the UAV Flight Mode

      @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
      Loiter, RTL, or Circle
  */
  void SetMode(std::string mode);

  /**
      Override the RC value of the transmitter

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

private:
  ros::NodeHandle nh_simple_control;
  ros::ServiceClient sc_arm, sc_land, sc_mode;
  ros::Publisher pub_override_rc, pub_setpoint_position;
};

#endif
