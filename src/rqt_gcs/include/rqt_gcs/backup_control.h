#ifndef CONNECTION_MANAGER
#define CONNECTION_MANAGER

#include <ros/ros.h>
#include <std_msgs/Int32.h>

#define QUEUE_SIZE 10 //Message Queue size for publishers
#define DEF_NS "UAV"

class BackupManager
{
public:
  int id;
  BackupManager(int id);
  ~BackupManager();

  void PublishHeartbeat(const ros::TimerEvent& e);

private:
  ros::NodeHandle nh;
  ros::Publisher  pub_heartbeat;
  ros::Timer timer_heartbeat;

  std_msgs::Int32 heartbeat;
};

#endif
