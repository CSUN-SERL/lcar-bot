#ifndef CONNECTION_MANAGER
#define CONNECTION_MANAGER

#include <ros/ros.h>

#define DEF_NS "UAV"

class ConnectionManager
{
public:
  ConnectionManager();
  ~ConnectionManager();

private:
  ros::Publisher pub_connected;
};

#endif
