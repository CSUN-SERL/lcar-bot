#!/usr/bin/env python2
#encoding: UTF-8

import platform
import rospy
import roslaunch
from std_msgs.msg import String
from lcar_msgs.msg import *
from lcar_msgs.srv import *

#globals
machine_name = ""
vehicle_id = -1
gcs_init_response = False


def InitResponseCallback(msg):
    global machine_name
    global vehicle_id
    global gcs_init_response
    
#    if machine_name != msg.machine_name:
#        return
    
    rospy.loginfo("%s received operator initialization request." % machine_name)
    vehicle_id = msg.vehicle_id
    gcs_init_response = True
   
    
    
def LaunchNodes():
#    todo launch nodes using roslaunch api
        package="rqt_gcs"
        executable="lcarbot_isolated.launch"
        node = roslaunch.core.Node(package, executable)
        
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        
        process = launch.launch(node)
        print "halp"
    


def Main():
    global machine_name
    global vehicle_id
    global gcs_init_response
    
    machine_name = platform.node() #get the host name of this computer
    machine_name = "quad1"
    rospy.loginfo("Overriden machine_name: %s" % machine_name)
    
    rospy.init_node('init_node')
    rospy.Subscriber("/vehicle/init/response", InitResponse, InitResponseCallback)
    
    rospy.wait_for_service('/vehicle/init/request')
    rospy.wait_for_service('/vehicle/init/final_ack')
    
    sc_init_request = rospy.ServiceProxy('/vehicle/init/request', InitRequest)
    sc_init_final_ack = rospy.ServiceProxy('/vehicle/init/final_ack', InitFinalAck)
    
    try:
        ir_response = sc_init_request(machine_name)
        if ir_response.ack:
            rospy.loginfo("init request acknowledged, waiting for operator to signal startup")
            vehicle_id = ir_response.vehicle_id
            
            rate = rospy.Rate(10)
            while not gcs_init_response: 
                rate.sleep()

            ifa_response = sc_init_final_ack(vehicle_id)
            if ifa_response.startup:
                rospy.loginfo("init final ack successful. launching nodes.")
                LaunchNodes()
            else:
                rospy.loginfo("init final ack failed.")
                
        else:
            rospy.loginfo("init request failed")
        
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e
    
    
    
if __name__ == "__main__":
    try:
        Main()
    except rospy.ROSInterruptException:
        pass
    
