#ifndef CONNECTION_MANAGER
#define CONNECTION_MANAGER

#include <ros/ros.h>

#define QUEUE_SIZE 10 //Message Queue size for publishers
#define DEF_NS "UAV"

class ConnectionManager
{
public:
  ConnectionManager();
  ~ConnectionManager();

private:
  void PublishConnection();

  ros::NodeHandle nh;
  ros::Publisher  pub_connection;
};

#endif
