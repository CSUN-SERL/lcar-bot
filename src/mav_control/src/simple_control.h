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

class SimpleControl
{
public:
  SimpleControl();
  ~SimpleControl();

  /**
      Arm or disarm the UAV

      @param value Pass true for arm, false for disarm
  */
  static void Arm(bool value);

  /**
      Land the UAV
  */
  static void Land();

  /**
      Set the UAV Flight Mode

      @param mode Mode to Set: Choose from Stabilize, Alt Hold, Auto, Guided,
      Loiter, RTL, or Circle
  */
  static void SetMode(std::string mode);
};

#endif
