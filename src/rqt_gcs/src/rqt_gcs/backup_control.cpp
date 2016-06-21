#include <rqt_gcs/backup_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "backup_control");

  BackupManager manager(1);
  ros::Rate loop_rate(10); //10Hz

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

BackupManager::BackupManager(int id)  //Class constructor
{
  //Setup heartbeat timer
  timer_heartbeat = nh.createTimer(ros::Duration(0.1), &BackupManager::PublishHeartbeat, this);

  std::string ns = DEF_NS + std::to_string(id); //UAV Namespace
  pub_heartbeat = nh.advertise<std_msgs::Int32>(ns + "/heartbeat/uav", 0);
}

BackupManager::~BackupManager()
{
    //Default destructor
}

void BackupManager::PublishHeartbeat(const ros::TimerEvent& e)
{
  heartbeat.data++;
  pub_heartbeat.publish(heartbeat);
}
