#include <rqt_gcs/backup_control.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "backup_control");

  BackupControl control(1);
  ros::Rate loop_rate(10); //10Hz

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

BackupControl::BackupControl(int id)  //Class constructor
{
  //Setup heartbeat timers
  timer_heartbeat_uav   = nh.createTimer(ros::Duration(0.1), &BackupControl::PublishHeartbeat, this);
  timer_heartbeat_gcs   = nh.createTimer(ros::Duration(0.25), &BackupControl::AssumeControl, this);
  
  //this topic should be advertised under the /UAV* namsepace, eg. /UAV1/heartbeat/uav
  pub_heartbeat = nh.advertise<std_msgs::Int32>("heartbeat/uav", 0);
  pub_sp_position = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

  sub_heartbeat = nh.subscribe("heartbeat/gcs", 0, &BackupControl::ReceiveHeartbeat, this);
  sub_pos_local  = nh.subscribe("mavros/local_position/pose", 0, &BackupControl::LocalPosCallback, this);
}

BackupControl::~BackupControl()
{
    //Default destructor
}

void BackupControl::PublishHeartbeat(const ros::TimerEvent &e)
{
  heartbeat_uav.data++;
  pub_heartbeat.publish(heartbeat_uav);
}

void BackupControl::ReceiveHeartbeat(const std_msgs::Int32 &msg_hb)
{
    //Restart Timer
    timer_heartbeat_gcs.stop();
    timer_heartbeat_gcs.start();

    //Update previous position
    pose_prev = pose_local;

    //Reset connection loss time
    time_connection_loss = -1;
}

void BackupControl::AssumeControl(const ros::TimerEvent &e){

    if(time_connection_loss < 0) time_connection_loss = ros::Time::now().toSec();

    //Calculate duration of connection loss
    double duration = ros::Time::now().toSec() - time_connection_loss;

    if(duration < 5){
        //Create the message object
        geometry_msgs::PoseStamped position_stamped;

        //Update the message with the latest previous position
        position_stamped.pose = pose_prev;

        //Publish the message
        pub_sp_position.publish(position_stamped);
    }
    else{
        //Return to Launch
        //TODO: Implement ability to RTL
        ROS_WARN_STREAM("Lost connection for too long. Returning home.");
    }
}