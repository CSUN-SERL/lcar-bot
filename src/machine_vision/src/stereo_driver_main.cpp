#include <stereo_driver.h>
#include <camera_info_manager/camera_info_manager.h>


int main(int argc, char **argv){

    //ros inits
    ros::init(argc, argv, "stereo_driver");
    ros::NodeHandle nh;

    std::string calib_file_left  = "left.yaml";
    std::string calib_file_right = "right.yaml";
    if(argc >= 3){
        calib_file_left = argv[2];
        calib_file_left = argv [3];
    }

    int vendor_id  = strtol("0x2a0b", NULL, 0); // leopard imaging vendor id
    int product_id = strtol("0x00f5", NULL, 0); //  li usb3 stereo camera id

    StereoDriver camera(
        vendor_id,
        product_id,
        nh
    );

    camera.StartStream();
    
    while(nh.ok() && camera.IsOpen()){
        ros::spinOnce();
    }


    return 0;
}
