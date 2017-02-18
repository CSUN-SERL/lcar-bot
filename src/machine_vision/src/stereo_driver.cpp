#include <stereo_driver.h>
#include <sensor_msgs/fill_image.h>
#include <cv_bridge/cv_bridge.h>
#include <endian.h>

using namespace camera_info_manager;

StereoDriver::StereoDriver(int vendor_id, int product_id, ros::NodeHandle nh) :
    vendor_id(vendor_id),
    product_id(product_id),
    nh(nh),
    it(nh),
    open(false)
{
    CameraInfoManager cim_left(ros::NodeHandle(nh, "stereo_cam/left" ), "li_stereo_left" );
    CameraInfoManager cim_right(ros::NodeHandle(nh, "stereo_cam/right"), "li_stereo_right");
    
    cim_left.loadCameraInfo("package://machine_vision/calibrations/left.yaml");
    cim_right.loadCameraInfo("package://machine_vision/calibrations/right.yaml");

    ci_left.reset(new sensor_msgs::CameraInfo(cim_left.getCameraInfo()));
    ci_right.reset(new sensor_msgs::CameraInfo(cim_right.getCameraInfo()));
    
    pub_left = it.advertiseCamera( "stereo_cam/left/image_raw", 1);
    pub_right = it.advertiseCamera("stereo_cam/right/image_raw", 1);
}

StereoDriver::~StereoDriver(){}

bool StereoDriver::IsOpen(){ return open; }

void StereoDriver::StartStream()
{
    res = uvc_init(&ctx, NULL);

    if (res < 0)
    {
      uvc_perror(res, "uvc_init");
      exit( res );
    }
    puts("UVC initialized");
    res = uvc_get_device_list(ctx, &list);
    if (res < 0)
    {
      uvc_perror(res, "uvc_get_device_list");
      exit (res);
    }
    /* Locates the first attached UVC device, stores in dev */
    res = uvc_find_device(
        ctx, &dev,
        vendor_id, product_id, NULL); /* filter devices: vendor_id, product_id, "serial_num" */

    if (res < 0) 
    {
      open=false;
      uvc_perror(res, "uvc_find_device"); /* no devices found */
    } 
    else
    {
      puts("Device found");

      /* Try to open the device: requires exclusive access */
      res = uvc_open(dev, &devh);

      if (res < 0) {
        uvc_perror(res, "uvc_open"); /* unable to open device */
        printf("Unable to open device");
        open=false;
      }
      else
      {
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

        if (res < 0) 
        {
          open = false;
          uvc_perror(res, "get_mode"); /* device doesn't provide a matching stream */
        } 
        else 
        {

          open = false;
          res = uvc_start_streaming(devh, &ctrl, &StereoDriver::ImageCallbackAdapter, this, 0);

          if (res < 0) 
          {
            uvc_perror(res, "start_streaming"); /* unable to start stream */
          } 
          else 
          {
            open = true;

            puts("Streaming...");

            uvc_set_ae_mode(devh, 0); /* e.g., turn on auto exposure */

            //sleep(10); /* streamfor 10 seconds */
            while(nh.ok() && open)
                ros::spinOnce();

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
    /* We'll convert the image from YUV/JPEG to gray8, so allocate space */
    uvc_frame_t *uvc_left = uvc_allocate_frame(frame->width * frame->height);
    uvc_frame_t *uvc_right = uvc_allocate_frame(frame->width * frame->height);

    if (!uvc_left) 
    {
      printf("unable to allocate left frame!");
      return;
    }
    if (!uvc_right) 
    {
      printf("unable to allocate right frame!");
      return;
    }

    /* Do the BGR conversion */
    uvc_error_t ret_left = uvc_yuyv2y(frame, uvc_left);
    uvc_error_t ret_right = uvc_yuyv2uv(frame, uvc_right);

    if(ret_left)
    {
      uvc_perror(ret_left, "uvc_yuyv2y");
      uvc_free_frame(uvc_left);
      printf("Error with yuyv to y conversion: error code %d", ret_left);
      return;
    }

    if(ret_right)
    {
      uvc_perror(ret_right, "uvc_yuyv2uv");
      uvc_free_frame(uvc_right);
      printf("Error with yuyv to uv conversion: error code %d", ret_right);
      return;
    }
    
    cv::Mat mat_left(uvc_left->height, uvc_left->width, CV_8UC1, uvc_left->data, uvc_left->step);
    cv::Mat mat_right(uvc_right->height, uvc_right->width, CV_8UC1, uvc_right->data, uvc_right->step);
    
    sensor_msgs::ImagePtr msg_left = 
            cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_left).toImageMsg();
    
    sensor_msgs::ImagePtr msg_right = 
            cv_bridge::CvImage(std_msgs::Header(), "mono8", mat_right).toImageMsg();

    ros::Time time_stamp = ros::Time::now();
    ci_left->header.stamp = ci_right->header.stamp = time_stamp;
    msg_left->header.stamp = msg_right->header.stamp = time_stamp;
    msg_left->header.seq = msg_right->header.seq = frame->sequence;
    msg_left->is_bigendian = msg_right->is_bigendian = (__BYTE_ORDER == __BIG_ENDIAN);
    
    pub_left.publish(msg_left, ci_left);
    pub_right.publish(msg_right, ci_right);

    uvc_free_frame(uvc_left);
    uvc_free_frame(uvc_right);
}

void StereoDriver::ImageCallbackAdapter(uvc_frame_t *frame, void *ptr)
{
    StereoDriver *driver = static_cast<StereoDriver*>(ptr);
    driver->ImageCallback(frame);
}
