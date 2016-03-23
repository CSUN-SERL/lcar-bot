#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>

#define QUEUE_SIZE 10 //Message Queue size for publishers
#define DEF_NS "UAV"
#define X_LIMIT 1           // 1 meter from the origin
#define Y_LIMIT 0.5         // 0.5 meter from the origin
#define Z_LIMIT 1           // 1 meter from the origin
#define SAFETY_RADIUS 0.5   // Sphere with 0.5m radius
#define SPIKE_LIMIT 0.05    // Limit for spikes in data

ros::Publisher pub_mocap;
ros::Subscriber sub_vrpn;

geometry_msgs::Pose pose_prev;
bool pos_received = false;

void VrpnCallback(const geometry_msgs::PoseStamped& msg_pose);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "safe_mocap");

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~"); //Private NodeHandle for parameters

  ros::Rate loop_rate(120); //120Hz

  int id = 1;

  bool ok = private_nh.getParam("id", id);
  if(!ok) ROS_ERROR_STREAM("Couldn't read the id parameter!");

  std::string ns = DEF_NS + std::to_string(id); //UAV Namespace
  pub_mocap = nh.advertise<geometry_msgs::PoseStamped>(ns + "/mavros/mocap/pose",QUEUE_SIZE);
  sub_vrpn  = nh.subscribe("vrpn_client_node/" + ns + "/pose", QUEUE_SIZE, VrpnCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void VrpnCallback(const geometry_msgs::PoseStamped& msg_pose)
{
    geometry_msgs::PoseStamped msg_new = msg_pose;
    geometry_msgs::Point point = msg_pose.pose.position;

    if(!pos_received) {
        pose_prev.position.x = msg_pose.pose.position.x;
        pose_prev.position.y = msg_pose.pose.position.y;
        pose_prev.position.z = msg_pose.pose.position.z;
        pose_prev.orientation.z = msg_pose.pose.orientation.z;

        pos_received = true;
    }

    //Check for violation of capture volume limits
    if(!(std::abs(point.x) < X_LIMIT)) point.x = pose_prev.position.x;
    else if(!(std::abs(point.y) < Y_LIMIT)) point.y = pose_prev.position.x;
    else if(!(std::abs(point.z) < Z_LIMIT)) point.z = pose_prev.position.x;

    //Check for value spikes
    if(abs(abs(point.x)-abs(pose_prev.position.x)) > SPIKE_LIMIT) point.x = pose_prev.position.x;
    if(abs(abs(point.x)-abs(pose_prev.position.x)) > SPIKE_LIMIT) point.y = pose_prev.position.y;
    if(abs(abs(point.x)-abs(pose_prev.position.x)) > SPIKE_LIMIT) point.z = pose_prev.position.z;

    //Update the position value in mocap data
    msg_new.pose.position.x = point.x;
    msg_new.pose.position.y = point.y;
    msg_new.pose.position.z = point.z;

    pose_prev = msg_new.pose;

    pub_mocap.publish(msg_new);
}
