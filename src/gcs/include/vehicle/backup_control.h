#ifndef BACKUPCONTROL_H
#define BACKUPCONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>

class BackupControl
{
public:
  int id;
  BackupControl(int id);
  ~BackupControl();

private:

  void PublishHeartbeat (const ros::TimerEvent &e);
  void AssumeControl    (const ros::TimerEvent &e);
  void ReceiveHeartbeat (const std_msgs::Int32 &msg_hb);
  void LocalPosCallback (const geometry_msgs::PoseStamped& msg_pos) { pose_local = msg_pos.pose; }

  ros::NodeHandle nh;
  ros::Publisher  pub_heartbeat, pub_sp_position;
  ros::Subscriber sub_heartbeat, sub_pos_local;
  ros::Timer timer_heartbeat_uav;
  ros::Timer timer_heartbeat_gcs;
  double time_connection_loss = -1;

  std_msgs::Int32 heartbeat_uav;
  geometry_msgs::Pose pose_local, pose_prev;
};

#endif
