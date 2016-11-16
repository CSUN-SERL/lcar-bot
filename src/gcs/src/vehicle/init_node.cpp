
#include <ros/ros.h>
#include "lcar_msgs/InitResponse.h"
#include "lcar_msgs/InitRequest.h"

#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <fstream>


int vehicle_id;
std::string machine_name;
bool gcs_init_response = false;

void InitResponseCallback(const lcar_msgs::InitResponseConstPtr& msg);
void PrepEnv(int id);
void Timeout(const ros::TimerEvent& e);

void InitResponseCallback(const lcar_msgs::InitResponseConstPtr& msg)
{
    //TODO UNCOMMENT THIS after machine_name override is undone inside main()
//    if(machine_name != msg->machine_name || vehicle_id != msg->vehicle_id)
//        return;
    
    ROS_INFO_STREAM(machine_name << " received operator initialization request.");
    PrepEnv(vehicle_id);
    ros::shutdown();
}

void Timeout(const ros::TimerEvent &e)
{
    ROS_WARN_STREAM(machine_name << " initialization timed out after 2 minutes");
    PrepEnv(-1);
    ros::shutdown();
}

void PrepEnv(int id)
{   
    std::string file_path = std::string(getenv("HOME")) + "/vehicle_id";
    std::ofstream f(file_path);
    if(f.is_open())
    {
        ROS_WARN_STREAM("writing id: " << id << " to " << file_path);
        f << id;
        f.close();
    }
    else
        ROS_ERROR_STREAM("couldn't open" << file_path  << "file for writing");
}

int main(int argc, char ** argv) 
{
    ros::init(argc, argv, "init_node", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    
    ros::Timer init_timeout = nh.createTimer(ros::Duration(120), Timeout);
    
    ros::ServiceClient sc_init_request = nh.serviceClient<lcar_msgs::InitRequest>("/vehicle/init/request");
    ros::Subscriber sub_init_response = nh.subscribe("/vehicle/init/response", 2, InitResponseCallback);
    
    char host[HOST_NAME_MAX];
    gethostname(host, HOST_NAME_MAX);
    machine_name = std::string(host);
    machine_name = "quad1"; //  TODO REMOVE THIS temporary for testing purposes only
                            //  uncomment if() check in InitResponseCallback() above 
    
    lcar_msgs::InitRequest init_request;
    init_request.request.machine_name = machine_name;
    
    ros::Rate loop_rate(10);
    while(ros::ok() && !sc_init_request.call(init_request)) // wait for InitRequest service
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    if(init_request.response.ack)
    {
        vehicle_id = init_request.response.vehicle_id;
        ROS_INFO_STREAM(init_request.response.message);

        ros::spin();
    }
    else
        ROS_WARN_STREAM(init_request.response.message);
    
    return 0;
}
