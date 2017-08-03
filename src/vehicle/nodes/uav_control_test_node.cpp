#include "vehicle/uav_control.h"
#include <ros/ros.h>


gcs::UAVControl * quad;
float height = 1.5;
std::vector<geometry_msgs::Pose> waypoints_list;   
   
void runCallback(const ros::TimerEvent& e)
{
    quad->Run();
}

geometry_msgs::Pose setupWaypoint(double x, double y, double z, double yaw)//yaw is in degrees
{
   geometry_msgs::Pose target_waypoint;
   target_waypoint.position.x = x;
   target_waypoint.position.y = y;
   target_waypoint.position.z = z;
   double yaw_angle = angles::normalize_angle_positive(angles::from_degrees(yaw));//face left
   quaternionTFToMsg(tf::createQuaternionFromYaw(yaw_angle), target_waypoint.orientation);
   return target_waypoint;
}
void runPredictable()//DON'T OVERRIDE WITH PULL
{
    // PREDICTABLE AND SIMPLE//
   //////BEGIN FIRST BUILDING//////
   geometry_msgs::Pose target_ptMBR = setupWaypoint(-1.9,-1.3,height,90);
   
   geometry_msgs::Pose target_ptMBB = setupWaypoint(-2.8,-0.3,height,0);
   
   geometry_msgs::Pose target_ptMBL = setupWaypoint(-2,0.5,height,270);
   
   geometry_msgs::Pose target_ptMBT = setupWaypoint(-1.2,-0.25,height,180);
   
   //////BEGIN SECOND BUILDING///////
   geometry_msgs::Pose target_ptMMB1 = setupWaypoint(-1,-0.25,height,0);
   
   geometry_msgs::Pose target_ptMMB2 = setupWaypoint(-1.2,-0.25,height,0);
   
   geometry_msgs::Pose target_ptMML = setupWaypoint(-0.35,0.5,height,270);
   
   geometry_msgs::Pose target_ptMMT = setupWaypoint(0.5,-0.3,height,180);
   
   geometry_msgs::Pose target_ptMMR = setupWaypoint(-0.45,-1.0,height,90);
   
   ///////////////////BEGIN 3RD BUILDING//////
   geometry_msgs::Pose target_ptRML1 = setupWaypoint(-0.45,-1.2,height,270);   
   
   geometry_msgs::Pose target_ptRML2 =setupWaypoint(-0.45,-1.0,height,270);
   
   geometry_msgs::Pose target_ptRMB = setupWaypoint(-1.2,-1.7,height,0);
   
   geometry_msgs::Pose target_ptRMR1 = setupWaypoint(-0.35,-1.7,height,90);
   
   geometry_msgs::Pose target_ptRMR2 = setupWaypoint(-0.35,-2.0,height,90);
   
   geometry_msgs::Pose target_ptRMR3 = setupWaypoint(-0.35,-2.4,height,90);
   
   geometry_msgs::Pose target_ptRMT = setupWaypoint(0.5,-1.7,height,180);
   
   //////////4th Building///////////////////
    geometry_msgs::Pose target_ptRTB1 = setupWaypoint(0.65,-1.45,height,0);
   
    geometry_msgs::Pose target_ptRTB2 = setupWaypoint(0.3,-1.45,height,0);
    
   geometry_msgs::Pose target_ptRTR = setupWaypoint(1.25,-2.0,height,90);
   
   geometry_msgs::Pose target_ptRTT = setupWaypoint(2.1,-1.5,height,180);
   
   geometry_msgs::Pose target_ptRTL = setupWaypoint(1.3,-0.5,height,270);
    //////////5th Building///////////////////
   geometry_msgs::Pose target_ptMTR1 = setupWaypoint(1.25,-0.8,height,90);
   
   geometry_msgs::Pose target_ptMTR2 = setupWaypoint(1.25,-0.9,height, 90);
   
   geometry_msgs::Pose target_ptMTT = setupWaypoint(2.1,-0.15,height,180);
   
   geometry_msgs::Pose target_ptMTB = setupWaypoint(0.1,-0.15,height,0);
   
   geometry_msgs::Pose target_ptMTL = setupWaypoint(1.25,0.9,height,270);
   //////////6th Building///////////////////
   geometry_msgs::Pose target_ptLTR1 = setupWaypoint(1.25,0.8,height,90);
   
   geometry_msgs::Pose target_ptLTR2 = setupWaypoint(1.25,0.5,height,90);
   
   geometry_msgs::Pose target_ptLTT = setupWaypoint(1.9,1.25,height,180);
   
   geometry_msgs::Pose target_ptLTL = setupWaypoint(1.35,2.0,height,270);
   
   geometry_msgs::Pose target_ptLTB = setupWaypoint(0.5,1.35,height,0);
   
   //////////7th Building///////////////////
    geometry_msgs::Pose target_ptLMT1 = setupWaypoint(0.4,1.35,height,180);
    
    geometry_msgs::Pose target_ptLMT2 = setupWaypoint(0.7,1.35,height,180);
    
   geometry_msgs::Pose target_ptLML = setupWaypoint(-0.15,2.0,height,270);
   
   geometry_msgs::Pose target_ptLMR = setupWaypoint(-0.25,0.75,height,90);
   
   geometry_msgs::Pose target_ptLMB = setupWaypoint(-1.2,1.35,height,0);
   
   //////////8th Building///////////////////
    geometry_msgs::Pose target_ptLBT1 = setupWaypoint(-1.1,1.35,height,180);
    
   geometry_msgs::Pose target_ptLBT2 = setupWaypoint(-1.0,1.35,height,180);
   
   geometry_msgs::Pose target_ptLBR = setupWaypoint(-1.65,0.3,height,90);
   
   geometry_msgs::Pose target_ptLBB1 = setupWaypoint(-1.65,1.35,height,0);
   
   geometry_msgs::Pose target_ptLBB2 = setupWaypoint(-2.2,1.35,height,0);
   
   geometry_msgs::Pose target_ptLBL1 = setupWaypoint(-1.6,1.5,height,270);
   
   geometry_msgs::Pose target_ptLBL2 = setupWaypoint(-1.6,2.15,height,270);
   
   geometry_msgs::Pose target_ptLast = setupWaypoint(-1.9,-1.5,height,270);
   
   waypoints_list.push_back(target_ptMBR);
   waypoints_list.push_back(target_ptMBB);
   waypoints_list.push_back(target_ptMBL);
   waypoints_list.push_back(target_ptMBT);
   waypoints_list.push_back(target_ptMMB1);
   waypoints_list.push_back(target_ptMMB2);
   waypoints_list.push_back(target_ptMML);
   waypoints_list.push_back(target_ptMMT);
   waypoints_list.push_back(target_ptMMR);
   waypoints_list.push_back(target_ptRML1);
   waypoints_list.push_back(target_ptRML2);
   waypoints_list.push_back(target_ptRMB);
   waypoints_list.push_back(target_ptRMR1);
   waypoints_list.push_back(target_ptRMR2);
   waypoints_list.push_back(target_ptRMR3);
   waypoints_list.push_back(target_ptRMT); 
   waypoints_list.push_back(target_ptRTB1);
   waypoints_list.push_back(target_ptRTB2);
   waypoints_list.push_back(target_ptRTR);
   waypoints_list.push_back(target_ptRTT);
   waypoints_list.push_back(target_ptRTL);
   waypoints_list.push_back(target_ptMTR1);
   waypoints_list.push_back(target_ptMTR2);
   waypoints_list.push_back(target_ptMTT);
   waypoints_list.push_back(target_ptMTB);
   waypoints_list.push_back(target_ptMTL);
   waypoints_list.push_back(target_ptLTR1);
   waypoints_list.push_back(target_ptLTR2);
   waypoints_list.push_back(target_ptLTT);
   waypoints_list.push_back(target_ptLTL);
   waypoints_list.push_back(target_ptLTB);
   waypoints_list.push_back(target_ptLMT1);
   waypoints_list.push_back(target_ptLMT2); 
   waypoints_list.push_back(target_ptLML);
   waypoints_list.push_back(target_ptLMR);
   waypoints_list.push_back(target_ptLMB);
   waypoints_list.push_back(target_ptLBT1);   
   waypoints_list.push_back(target_ptLBT2);
   waypoints_list.push_back(target_ptLBR);
   waypoints_list.push_back(target_ptLBB1);
   waypoints_list.push_back(target_ptLBB2);
   waypoints_list.push_back(target_ptLBL1);
   waypoints_list.push_back(target_ptLBL2);
   waypoints_list.push_back(target_ptLast);
   
}

void runUnpredictable()
{
    //UNPREDICTABLE
   geometry_msgs::Pose target_ptRMB = setupWaypoint(-1.8,-1.7,height,0);//RMB (RIGHT MID BOTTOM)
   
   geometry_msgs::Pose target_pt2 = setupWaypoint(-1.9,-1.3,height,90);//MBR (MID BOTTOM RIGHT)
   
   geometry_msgs::Pose target_pt3 = setupWaypoint(-2.9,-0.25,height,0); //MBB
   
   geometry_msgs::Pose target_pt4 = setupWaypoint(-2.0,0.5,height,270); //MBL
   
   geometry_msgs::Pose target_pt4_unprd = setupWaypoint(-2,0.5,height,90); //MBL
   
   geometry_msgs::Pose target_pt5_1 = setupWaypoint(-1.2,-0.25,height,180); //MBT
   
   geometry_msgs::Pose target_pt5_2 = setupWaypoint(-1.2,-0.25,height,0); //MMB
   
   geometry_msgs::Pose target_pt6 = setupWaypoint(-0.28,0.55,height,270); //MML
   
   geometry_msgs::Pose target_pt7 = setupWaypoint(-0.28,0.55,height,90);//LMR
   
   geometry_msgs::Pose target_pt8 = setupWaypoint(-0.9,1.35,height,0);
   
   geometry_msgs::Pose target_pt9_1 = setupWaypoint(-0.15, 2.2,height,270);
   
   geometry_msgs::Pose target_pt9_2 = setupWaypoint(0.6,1.34,height,180);
   
   geometry_msgs::Pose target_pt10 = setupWaypoint(0.6,1.34,height,0);
   
   geometry_msgs::Pose target_pt11 = setupWaypoint(1.3,2.1,height,270);
   
   geometry_msgs::Pose target_pt12 = setupWaypoint(2.1,1.3,height,180);
   
   geometry_msgs::Pose target_pt13_1 = setupWaypoint(1.25,0.5,height,270);
   
   geometry_msgs::Pose target_pt13_2 = setupWaypoint(1.25,0.5,height,90);
   
   geometry_msgs::Pose target_pt14 = setupWaypoint(0.4,-0.3,height,180);
   
   geometry_msgs::Pose target_pt15 = setupWaypoint(0.4,-0.3,height,0);
   
   geometry_msgs::Pose target_pt16 = setupWaypoint(-0.4,-1.0,height,90);
   
   geometry_msgs::Pose target_pt17_1 = setupWaypoint(-0.4,-1.0,height,270);
   
   geometry_msgs::Pose target_pt17_2 = setupWaypoint(0.5,-1.6,height,180);//RMT
   
   geometry_msgs::Pose target_pt18 = setupWaypoint(0.5,-1.6,height,0);//RTB
   
   geometry_msgs::Pose target_pt19 = setupWaypoint(1.3,-2.1,height,90);//RTR
   
   geometry_msgs::Pose target_pt20 = setupWaypoint(2.0,-1.5,height,180);//RTT
   
   geometry_msgs::Pose target_pt21_1 = setupWaypoint(1.25,-0.8,height,270);//RTL
   
   geometry_msgs::Pose target_pt21_2 = setupWaypoint(1.25,-0.8,height,90);//MTR
   
   geometry_msgs::Pose target_pt22 = setupWaypoint(1.9,-0.15,height,180);//MTT
   
   geometry_msgs::Pose target_pt23 = setupWaypoint(2.1,1.35,height,180);//LTT
   
   geometry_msgs::Pose target_pt24 = setupWaypoint(-1.5,2.15,height,270);
   
   geometry_msgs::Pose target_pt25_1 = setupWaypoint(-2.2,1.35,height,0);
   
   geometry_msgs::Pose target_pt25_2 = setupWaypoint(-2.1,-1.6,height,90);
   
   geometry_msgs::Pose target_pt26 = setupWaypoint(-0.35,-2.4,height,90);
   
   geometry_msgs::Pose target_pt27 = setupWaypoint(-1.7,-1.9,height,90);
   
   waypoints_list.push_back(target_ptRMB);
   waypoints_list.push_back(target_pt2);
   waypoints_list.push_back(target_pt3);
   waypoints_list.push_back(target_pt4);
   waypoints_list.push_back(target_pt5_1);
   waypoints_list.push_back(target_pt5_2);
   waypoints_list.push_back(target_pt6);
   waypoints_list.push_back(target_pt7);
   waypoints_list.push_back(target_pt8);
   waypoints_list.push_back(target_pt9_1);
   waypoints_list.push_back(target_pt9_2);
   waypoints_list.push_back(target_pt10);
   waypoints_list.push_back(target_pt11);
   waypoints_list.push_back(target_pt12);
   waypoints_list.push_back(target_pt13_1);   
   waypoints_list.push_back(target_pt13_2);
   waypoints_list.push_back(target_pt14);
   waypoints_list.push_back(target_pt15);
   waypoints_list.push_back(target_pt16);
   waypoints_list.push_back(target_pt17_1);
   waypoints_list.push_back(target_pt17_2);*/
   waypoints_list.push_back(target_pt18);
   waypoints_list.push_back(target_pt19);
   waypoints_list.push_back(target_pt20);
   waypoints_list.push_back(target_pt21_1);
   waypoints_list.push_back(target_pt21_2);
   waypoints_list.push_back(target_pt22);
   waypoints_list.push_back(target_pt23);
   waypoints_list.push_back(target_pt24);
   waypoints_list.push_back(target_pt25_1);
   waypoints_list.push_back(target_pt25_2);
   waypoints_list.push_back(target_pt26); 
   waypoints_list.push_back(target_pt27);
   
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "uav_control_test");
   ros::NodeHandle nh;
   
   
   quad = new gcs::UAVControl(2000);
   
   ros::Timer timer = nh.createTimer(ros::Duration(0.1), runCallback);
   
   ros::AsyncSpinner spinner(2);
   spinner.start();
  
   ros::Rate(0.3).sleep(); 
   
   runPredictable();
    
   quad->SetMission(waypoints_list);
   
   quad->StartMission();
   
   quad->EnableOffboard();
   
   ros::waitForShutdown();
   
   return 0;
}