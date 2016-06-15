#include <stereo_driver.h>

using namespace camera_info_manager;

StereoDriver::StereoDriver(
    int vendor_id,
    int product_id,
    ros::NodeHandle nh
) :
    vendor_id_(vendor_id),
    product_id_(product_id),
    nh_(nh),
    it_(nh),
    cinfo_left_ (ros::NodeHandle(nh_, "stereo_cam/left" ), "li_stereo_left" ),
    cinfo_right_(ros::NodeHandle(nh_, "stereo_cam/right"), "li_stereo_right"),
    open_(false),
    stereo_image_id_(0)
{
    cinfo_left_.loadCameraInfo("package://machine_vision/calibrations/left.yaml");
    cinfo_right_.loadCameraInfo("package://machine_vision/calibrations/right.yaml");

    pub_left_  = it_.advertiseCamera( "stereo_cam/left/image_raw", 1);
    pub_right_ = it_.advertiseCamera("stereo_cam/right/image_raw", 1);
    pub_rgb_   = it_.advertise("stereo_cam/rgb/image_raw", 1);
}

StereoDriver::~StereoDriver(){}

bool StereoDriver::IsOpen(){ return open_; }

void StereoDriver::StartStream(){

    res = uvc_init(&ctx, NULL);

    if (res < 0) {
      uvc_perror(res, "uvc_init");
      exit( res );
    }
    puts("UVC initialized");
    res = uvc_get_device_list(ctx, &list);
    if (res < 0) {
      uvc_perror(res, "uvc_get_device_list");
      exit (res);
    }
    /* Locates the first attached UVC device, stores in dev */
    res = uvc_find_device(
        ctx, &dev,
        vendor_id_, product_id_, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

    if (res < 0) {
      open_=false;
      uvc_perror(res, "uvc_find_device"); /* no devices found */
    } else {
      puts("Device found");

      /* Try to open the device: requires exclusive access */
      res = uvc_open(dev, &devh);

      if (res < 0) {
        uvc_perror(res, "uvc_open"); /* unable to open device */
        printf("Unable to open device");
        open_=false;
      } else {
        puts("Device opened");

        /* Print out a message containing all the information that libuvc
         * knows about the device */
        uvc_print_diag(devh, stderr);

        /* Try to negotiate a 640x480 30 fps YUYV stream profile */
        res = uvc_get_stream_ctrl_format_size(
            devh, &ctrl, /* result stored in ctrl */
            UVC_FRAME_FORMAT_YUYV, /* YUV 422, aka YUV 4:2:2. try _COMPRESSED */
            640, 480, 30 /* width, height, fps */
        );

        /* Print out the result */
        uvc_print_stream_ctrl(&ctrl, stderr);

        if (res < 0) {
          open_ = false;
          uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
        } else {
          /* Start the video stream. The library will call user function cb:
           *   cb(frame, (void*) 12345)
           */

          open_ = false;
          res = uvc_start_streaming(devh, &ctrl, &StereoDriver::ImageCallbackAdapter, this, 0);

          if (res < 0) {
            uvc_perror(res, "start_streaming"); /* unable to start stream */
          } else {
            open_ = true;

            puts("Streaming...");

            uvc_set_ae_mode(devh, 0); /* e.g., turn on auto exposure */

            //sleep(10); /* stream for 10 seconds */
            while(nh_.ok() && open_){
                ros::spinOnce();
            }

            /* End the stream. Blocks until last callback is serviced */
            uvc_stop_streaming(devh);
            puts("Done streaming.");
          }
        }

        /* Release our handle on the device */
        uvc_close(devh);
        puts("Device closed");
      }

      /* Release the device descriptor */
      uvc_unref_device(dev);
    }

    /* Close the UVC context. This closes and cleans up any existing device handles,
     * and it closes the libusb context if one was not provided. */
    uvc_exit(ctx);
    puts("UVC exited");
}

void StereoDriver::ImageCallback(uvc_frame_t *frame){
    //ros::Rate loop_rate(30);
    //loop_rate.sleep();
    frame->frame_format = UVC_FRAME_FORMAT_YUYV;

    //std::cout << "Frame format" << std::endl;
    //std::cout << frame->frame_format << std::endl;

    uvc_frame_t *greyLeft;
    uvc_frame_t *greyRight;
    
    uvc_error_t retLeft;
    uvc_error_t retRight;

    /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
    greyLeft = uvc_allocate_frame(frame->width * frame->height);
    greyRight = uvc_allocate_frame(frame->width * frame->height);

    if (!greyLeft) {
      printf("unable to allocate grey left frame!");
      return;
    }
    if (!greyRight) {
      printf("unable to allocate grey right frame!");
      return;
    }

    sensor_msgs::CameraInfoPtr ci_left(new sensor_msgs::CameraInfo(cinfo_left_.getCameraInfo()));

    sensor_msgs::CameraInfoPtr ci_right(new sensor_msgs::CameraInfo(cinfo_right_.getCameraInfo()));

    ros::Time time_stamp = ros::Time::now();

    ci_left->header.stamp = ci_right->header.stamp = time_stamp;

    /* Do the BGR conversion */
    retLeft = uvc_yuyv2y(frame, greyLeft);
    retRight = uvc_yuyv2uv(frame, greyRight);

    if (retLeft) {
      uvc_perror(retLeft, "uvc_yuyv2y");
      uvc_free_frame(greyLeft);
      printf("Error with yuyv to y conversion");
      return;
    }

    if (retRight) {
      uvc_perror(retRight, "uvc_yuyv2uv");
      uvc_free_frame(greyRight);
      printf("Error with yuyv to uv conversion");
      return;
    }

    cv::Mat left (greyLeft->height, greyLeft->width, CV_8UC1, greyLeft->data);
    cv::Mat right (greyRight->height, greyRight->width, CV_8UC1, greyRight->data);;

    if(!left.empty() && !right.empty()){
        sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(),
                                                "mono8", left).toImageMsg();

        sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(),
                                                "mono8", right).toImageMsg();
        msg_left->header.stamp = time_stamp;
        msg_right->header.stamp = time_stamp;

        msg_left->header.seq = msg_right->header.seq = stereo_image_id_;

        pub_left_.publish(msg_left, ci_left);
        pub_right_.publish(msg_right, ci_right);

        stereo_image_id_++;
    }

    uvc_free_frame(greyLeft);
    uvc_free_frame(greyRight);
}

void StereoDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr){
    StereoDriver *driver = static_cast<StereoDriver*>(ptr);
    driver->ImageCallback(frame);
}
