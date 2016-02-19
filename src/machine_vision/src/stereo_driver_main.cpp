#include <stereo_driver.h>
#include <camera_info_manager/camera_info_manager.h>


int main(int argc, char **argv){

    //ros inits
    ros::init(argc, argv, "stereo_driver");
    ros::NodeHandle nh;

    std::string calib_file = "calibration1.yaml";
    if(argc >= 2)
        calib_file = argv[2];

    CameraInfoManager cinfo(nh,
        "li_stereo",
        ("package://machine_vision/calibrations/" + calib_file)
    );

    int vendor_id = strtol("0x2a0b", NULL, 0); // leopard imaging vendor id
    int product_id = strtol("0x00f5", NULL,0); //  li usb3 stereo camera id

    CameraDriver camera( vendor_id,
                                        product_id,
                                        nh,
                                        cinfo
    );

    //camera.stream();

    ros::spin();

    return 0;
}
