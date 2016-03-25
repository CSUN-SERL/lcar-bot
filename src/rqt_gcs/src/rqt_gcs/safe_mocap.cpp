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

ros::Publisher pub_mocap, pub_violation;
ros::Subscriber sub_vrpn;

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
  pub_violation = nh.advertise<geometry_msgs::Vector3>(ns + "/mocap/violation",QUEUE_SIZE);
  sub_vrpn  = nh.subscribe("vrpn_client_node/" + ns + "/pose", QUEUE_SIZE, VrpnCallback);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

}

void VrpnCallback(const geometry_msgs::PoseStamped& msg_pose)
{
    geometry_msgs::Vector3 msg_violation;

    //Check for violation of capture volume limits
    if(std::abs(msg_pose.pose.position.x) > X_LIMIT) msg_violation.x = 1;
    else if(std::abs(msg_pose.pose.position.y) > Y_LIMIT) msg_violation.y = 1;
    else if(std::abs(msg_pose.pose.position.z) > Z_LIMIT) msg_violation.z = 1;
    pub_violation.publish(msg_violation);

    pub_mocap.publish(msg_pose);
}
