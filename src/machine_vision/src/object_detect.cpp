#include "libuvc/libuvc.h"

#include <opencv/cv.h>
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>


#include <iostream>
#include <thread>

using namespace std;
using namespace cv;
using namespace cv::ml;

image_transport::Publisher pubLeft;
image_transport::Publisher pubRight;
image_transport::Publisher pubRGB;

Ptr<SVM> svm;
vector< float > hog_detector;
Scalar reference(0, 255, 0);
HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
vector <Rect> objects;
//vector <cv::Mat > images;
//Mat src;
//bool start_detection = false;

int frame_id;

//Draw detected objects on a window
void DrawLocations(cv::Mat & img, const std::vector< Rect > & found, const Scalar & color) {
    if (!found.empty()) {
       std::vector< Rect >::const_iterator loc = found.begin();
       std::vector< Rect >::const_iterator end = found.end();
       for (; loc != end; ++loc) {
           rectangle(img, *loc, color, 2);
          }
    }
}
/////////////////////////////////////////////////////////////////////////////

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_bridge::CvImagePtr cv_img_ptr = cv_bridge::toCvCopy(msg, "mono8");

    objects.clear();
    
    //if(frame_id++ % 10 == 0){
        hog.detectMultiScale(cv_img_ptr->image, objects, true);
        DrawLocations(cv_img_ptr->image, objects, Scalar(0,255,0));
    //}

    imshow("view", cv_img_ptr->image);
    cv::waitKey(1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error in image subscriber callback: %s", e.what());
  }
}

//extract svm weights and store in a float vector
void GetSvmDetector(const Ptr<SVM>& svm, vector< float > & hog_detector) {
    // get the support vectors
    Mat sv = svm->getSupportVectors();
    const int sv_total = sv.rows;
    // get the decision function
    cv::Mat alpha;
    cv::Mat svidx;
    double rho = svm->getDecisionFunction(0, alpha, svidx);
    svidx.reshape(1, 1);
    hog_detector.clear();
    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof (hog_detector[0]));
    hog_detector[sv.cols] = (float) -rho;
}


////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv){
   ros::init(argc, argv, "object_detection", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  cv::namedWindow("view");
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  std::string topic;
  if(argc >= 2)
      topic = argv[1];
  else 
      topic = "stereo_cam/left/image_raw";
  
  string svmPath = ros::package::getPath("machine_vision");
  svm = StatModel::load<SVM>( svmPath+ "/LinearHOG.xml" );

  GetSvmDetector(svm, hog_detector);

  hog.winSize = Size(128, 256);
  hog.setSVMDetector(hog_detector);


  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
