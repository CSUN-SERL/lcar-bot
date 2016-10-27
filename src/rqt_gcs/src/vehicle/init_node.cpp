
#include <ros/ros.h>
#include "lcar_msgs/InitFinalAck.h"
#include "lcar_msgs/InitResponse.h"
#include "lcar_msgs/InitRequest.h"

#include <stdlib.h>
#include <unistd.h>

int vehicle_id;
std::string machine_name;
ros::ServiceClient *sc_final_ack;
bool gcs_response = false;

void StartupNodes();

void InitResponseCallback(const lcar_msgs::InitResponseConstPtr& msg)
{
    if(machine_name != msg->machine_name)
        return;
    
    ROS_INFO_STREAM("received operator initialization request");
    
    gcs_response = true;
    
    lcar_msgs::InitFinalAck ack;
    ack.request.vehicle_id = msg->vehicle_id;
    
    if(sc_final_ack->call(ack))
    {
        ROS_INFO_STREAM("init_node for " << machine_name << " starting nodes");
        StartupNodes();
    }
    else
    {
        ROS_INFO_STREAM("init node for " << machine_name << " failed");
        gcs_response = false;
    }
}

void StartupNodes()
{
    //todo properly launch nodes for given vehicle type
    system("lcar_bot");
}

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "init_node");
    ros::NodeHandle nh;
    
    ros::ServiceClient sc_init_request = nh.serviceClient<lcar_msgs::InitRequest>("vehicle/init/request");
    ros::ServiceClient sc_temp = nh.serviceClient<lcar_msgs::InitFinalAck>("vehicle/init/final_ack");
    sc_final_ack = &sc_temp;
    ros::Subscriber sub_init_response = nh.subscribe("vehicle/init/response", 2, InitResponseCallback);
    
    char host[HOST_NAME_MAX];
    gethostname(host, HOST_NAME_MAX);
    machine_name = std::string(host);
    
    lcar_msgs::InitRequest req;
    req.request.machine_name = machine_name;
    if(sc_init_request.call(req))
        ROS_INFO_STREAM("init_node for " << machine_name << " request acknowledged");
    else
        ROS_WARN_STREAM("init_node for " << machine_name << " request denied");
    
    ros::Rate loop_rate(10);
    while(ros::ok() && !gcs_response)
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}
