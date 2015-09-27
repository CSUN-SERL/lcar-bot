#ifndef SIMPLE_CONTROL
#define SIMPLE_CONTROL

#include <ros/ros.h>
#include <termios.h>
#include <stdio.h>
#include <mavros/mavros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>

class SimpleControl
{
public:
  SimpleControl();
  ~SimpleControl();
};

#endif
