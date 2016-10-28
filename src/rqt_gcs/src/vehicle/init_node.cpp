
#include <ros/ros.h>
#include "lcar_msgs/InitFinalAck.h"
#include "lcar_msgs/InitResponse.h"
#include "lcar_msgs/InitRequest.h"

#include <stdlib.h>
#include <unistd.h>

int vehicle_id;
std::string machine_name;
ros::ServiceClient sc_final_ack;
bool gcs_init_response = false;
bool gcs_final_ack = false;

void StartupNodes();

void InitResponseCallback(const lcar_msgs::InitResponseConstPtr& msg)
{
    if(machine_name != msg->machine_name)
        return;
    
    ROS_INFO_STREAM(machine_name << " received operator initialization request.");
    
    vehicle_id = msg->vehicle_id;
    gcs_init_response = true;
}

void StartupNodes()
{
    //todo properly launch nodes for given vehicle type
    std::string cmd("lcar-bot " + std::to_string(vehicle_id));
    system("printenv");
    system(cmd.c_str());
}

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "init_node");
    ros::NodeHandle nh;
    
    ros::ServiceClient sc_init_request = nh.serviceClient<lcar_msgs::InitRequest>("vehicle/init/request");
    sc_final_ack = nh.serviceClient<lcar_msgs::InitFinalAck>("vehicle/init/final_ack");
    ros::Subscriber sub_init_response = nh.subscribe("vehicle/init/response", 2, InitResponseCallback);
    
    char host[HOST_NAME_MAX];
    gethostname(host, HOST_NAME_MAX);
    machine_name = std::string(host);
    machine_name = "quad1";

    lcar_msgs::InitRequest req;
    req.request.machine_name = machine_name;
    if(sc_init_request.call(req))
        ROS_INFO_STREAM(machine_name << " init_request acknowledged");
    else
        ROS_WARN_STREAM(machine_name << " init_request denied");

    ros::Rate loop_rate(2);
    while(ros::ok() && !gcs_init_response) // gcs_init_response update in callback
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    lcar_msgs::InitFinalAck ack;
    ack.request.vehicle_id = vehicle_id;
    if(sc_final_ack.call(ack))
    {
        ROS_INFO_STREAM(machine_name << " init_final_ack: starting nodes");
        gcs_final_ack = true;
        StartupNodes();
    }
    else
        ROS_INFO_STREAM(machine_name << " init_final_ack: failed");


    return 0;
}
