
#include <ros/ros.h>
#include "lcar_msgs/InitFinalAck.h"
#include "lcar_msgs/InitResponse.h"
#include "lcar_msgs/InitRequest.h"

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>

int vehicle_id;
std::string machine_name;
ros::ServiceClient sc_final_ack;
ros::Subscriber sub_init_response;
bool gcs_init_response = false;

void PrepEnv();

void InitResponseCallback(const lcar_msgs::InitResponseConstPtr& msg)
{
//    if(machine_name != msg->machine_name || vehicle_id != msg->vehicle_id)
//        return;
    
    ROS_INFO_STREAM(machine_name << " received operator initialization request.");
    
    vehicle_id = msg->vehicle_id;
    gcs_init_response = true;
}

void PrepEnv(int id)
{
    std::string home_path= getenv("HOME");
    std::ofstream f(home_path + "/vehicle_id");
    if(f.is_open())
    {
        f << id;
        f.close();
    }
    else
        ROS_ERROR_STREAM("couldnt't open file for writing");
}

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "init_node",ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    ros::ServiceClient sc_init_request = nh.serviceClient<lcar_msgs::InitRequest>("vehicle/init/request");
    sc_final_ack = nh.serviceClient<lcar_msgs::InitFinalAck>("vehicle/init/final_ack");
    sub_init_response = nh.subscribe("vehicle/init/response", 2, InitResponseCallback);
    
    char host[HOST_NAME_MAX];
    gethostname(host, HOST_NAME_MAX);
    machine_name = std::string(host);
    machine_name = "quad1";

    lcar_msgs::InitRequest req;
    req.request.machine_name = machine_name;
    if(sc_init_request.call(req))
    {
        ROS_INFO_STREAM(machine_name << " init_request acknowledged");
        
        ros::Rate loop_rate(2);
        while(ros::ok() && !gcs_init_response) // gcs_init_response update in callback
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
        
        PrepEnv(vehicle_id);
    }
    else
    {
        ROS_WARN_STREAM(machine_name << " init_request denied");
        PrepEnv(-1);
    }

    return 0;
}
