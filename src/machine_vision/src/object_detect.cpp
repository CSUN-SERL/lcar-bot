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

Ptr<SVM> svm;
vector< float > hog_detector;
Scalar reference(0, 255, 0);
HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9)
//vector <cv::Mat > images;
Mat src;
//bool start_detection = false;
vector<Mat> trainingdata;
const Size & img_size = Size(128, 256);

int frame_id;
double avg;


/////////////////////////////////////////////////////////////////////////////
void HogFeatureExtraction(Mat ImgMat) {
    HOGDescriptor d;
    d.winSize = img_size;
    vector< Point > location;
    vector< float > descriptors;
    d.compute(ImgMat, descriptors, Size(8, 8), Size(0, 0), location);
    Mat hog_features = Mat(descriptors).clone();
    if (!hog_features.empty()){
        trainingdata.push_back(hog_features.reshape(1,1));
    }
}

void ConvertToMl(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData) {
    //--Convert data
    const int rows = (int) train_samples.size();
    const int cols = (int) std::max(train_samples[0].cols, train_samples[0].rows);
    cv::Mat tmp(1, cols, CV_32FC1); //< used for transposition if needed
    trainData = cv::Mat(rows, cols, CV_32FC1);
    vector< Mat >::const_iterator itr = train_samples.begin();
    vector< Mat >::const_iterator end = train_samples.end();
    for (int i = 0; itr != end; ++itr, ++i) {
        CV_Assert(itr->cols == 1 ||
                itr->rows == 1);
        if (itr->cols == 1) {
            transpose(*(itr), tmp);
            tmp.copyTo(trainData.row(i));
        } else if (itr->rows == 1) {
            itr->copyTo(trainData.row(i));
        }
    }
}

//Draw detected objects on a window
void DrawLocations(cv::Mat & img, const std::vector< Rect > & found, const vector<double> weights, const Scalar & color) {
    if (!found.empty()) {
       std::vector< Rect >::const_iterator loc = found.begin(); // Rect for detectMultiscale(), Point for detect()
       std::vector< Rect >::const_iterator end = found.end();   // same as above
       for (int i = 0; loc != end && i < weights.size(); ++loc) {
           //cout << weights[i] << endl;
           //if(weights[i] > 0.8)
//               rectangle(img, *loc, *loc, color, 2); //Point implementation used with hog.detect
               rectangle(img, *loc, color, 2); //Rect implementation used with hog.detectMultiScale
          }
    }
}

void ObjectCategorize(cv::Mat image){
  vector <double> weights;
  vector <Rect> detected_objects;
  vector< Rect > door_objects;
  hog.detectMultiScale(
          image,
          detected_objects,
          weights
          ,0         // hit threshold
          ,Size(0,0) // step size
          ,Size(0,0) // padding
          ,1.05       // scale factor
          ,2       // final threshold
          ,false      // use mean shift grouping?
  );
  for(int i = 0; i < detected_objects.size(); i++){
     Rect windows(detected_objects.at(i).x, detected_objects.at(i).y, detected_objects.at(i).width, detected_objects.at(i).height);
     Mat Roi = image(windows);
     Mat extract, res, train_data;
     resize(Roi, extract, img_size);
     HogFeatureExtraction(extract);
     ConvertToMl(trainingdata, train_data);
     trainingdata.clear();
     svm->predict(train_data, res, 4);
     for (int j = 0; j < res.rows; j++) {
            if (res.at<float>(j, 0) == 0) {
                door_objects.push_back(detected_objects.at(i));
            }
    }
    res.release();
    train_data.release();
    extract.release();
  }
  DrawLocations(src, door_objects, weights, Scalar(0,255,0));

}

void ImageShow(){
  imshow("view", src);
  cv::waitKey(1);
}
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat image;
    if(frame_id++ % 1 == 0){
        image = cv_bridge::toCvCopy(msg, "mono8")->image;
        src = image.clone();
        thread thread_1(ObjectCategorize, image);
        thread thread_2(ImageShow);
        if(thread_1.joinable()){
          thread_1.detach();
        }
        if(thread_2.joinable()){
          thread_2.join();
        }
//        vector <Point> detected_objects;
        double t = (double)cvGetTickCount();
//        hog.detectMultiScale(image, detected_objects, false);



//        hog.detect(
//                image,
//                detected_objects,
//                weights
//                //,0
//                //,Size(8,8)
//                //,Size(0,0)
//                //,1.5
//                //,1.0
//                //,true
//        );
        t = cvGetTickCount() - t;
        t = t / ((double)cvGetTickFrequency()*1000);
        avg += t;
        if(frame_id %10 == 0 ){
            cout << "detection time = " << avg / 10 << endl;
            avg = 0;
        }
    }
    else
        image = cv_bridge::toCvShare(msg, "mono8")->image;
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
    //const int sv_total = sv.rows;
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
  svm = StatModel::load<SVM>( svmPath + "/LinearHOG.xml" );

  GetSvmDetector(svm, hog_detector);

  hog.winSize = Size(128, 256);
  hog.setSVMDetector(hog_detector);


  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
