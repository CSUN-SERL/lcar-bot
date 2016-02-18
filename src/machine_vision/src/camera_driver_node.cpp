// #include "camera_driver.h"
//
//
// CameraDriver::CameraDriver(){
//     product_id = 0;
//     vendor_id = 0;
// }
//
// CameraDriver::CameraDriver(string vend_id="", string prod_id=""){
//
//     vendor_id_ = hexString2Int(vent_id);
//     product_id_ = hexString2Int(prod_id);
//     open(vendor_id, product_id);
// }
//
// unsigned int CameraDriver::hexString2Int(string hex_string){
//     int temp;
//     if(hex_string == "")
//         temp = 0;
//     else{
//         static stringstream ss;
//         ss << std::hex << hex_string;
//         ss >> temp;
//     }
//     return temp;
// }


#include "libuvc/libuvc.h"

#include <opencv/cv.h>
#include "opencv2/highgui.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <sstream> // for converting the command line parameter to integer#
#include <stdlib.h>
#include <camera_info_manager/camera_info_manager.h>

image_transport::Publisher pubLeft;
image_transport::Publisher pubRight;
image_transport::Publisher pubRGB;

int main(int argc, char ** argv){

    ros::init(argc, argv, "stereo_cam");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    pubLeft = it.advertise( "stereo_cam/left/image_raw", 1);
    pubRight = it.advertise("stereo_cam/right/image_raw", 1);
    pubRGB = it.advertise(  "stereo_cam/rgb/image_raw", 1);





}
