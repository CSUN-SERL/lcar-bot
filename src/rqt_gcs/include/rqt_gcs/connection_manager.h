#ifndef CONNECTION_MANAGER
#define CONNECTION_MANAGER

#include <ros/ros.h>
#include <std_msgs/Int8.h>

#define QUEUE_SIZE 10 //Message Queue size for publishers
#define DEF_NS "UAV"

class ConnectionManager
{
public:
  static int id;
  ConnectionManager();
  ~ConnectionManager();

  void PublishConnection();

private:
  ros::NodeHandle nh;
  ros::Publisher  pub_connection;
};

#endif
