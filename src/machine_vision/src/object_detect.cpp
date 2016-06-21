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
#include <lcar_msgs/Door.h>
 
using namespace std;
using namespace cv;
using namespace cv::ml;

ros::Publisher pub_mat_;
ros::Publisher pub_query_;

Ptr<SVM> svm_;
vector< float > hog_detector_;
HOGDescriptor hog_; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9)
vector<Mat> histograms;
//cv::Mat src;
vector< Mat > training_data_;
vector< cv::Mat > img_array_;
const Size & img_size_ = Size(128, 256);

int door_count = 0;
int img_iterator = 0;
double compare_hist_threshold = 1; 

/////////////////////////////////////////////////////////////////////////////
void HogFeatureExtraction(Mat ImgMat) {
    HOGDescriptor d;
    d.winSize = img_size_;
    vector< Point > location;
    vector< float > descriptors;
    d.compute(ImgMat, descriptors, Size(8, 8), Size(0, 0), location);
    Mat hog_features = Mat(descriptors).clone();
    if (!hog_features.empty()){
        training_data_.push_back(hog_features.reshape(1,1));
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
       std::vector< Rect >::const_iterator loc = found.begin(); // Rect for detectMultiscale(), Point for detect()
       std::vector< Rect >::const_iterator end = found.end();   // same as above

       for (int i = 0; loc != end; ++loc) {
               //Rect implementation used with hog.detectMultiScale
               rectangle(img, *loc, color, 2); 
        }
}

void ObjectCategorize(const cv::Mat& gray_image, cv::Mat& color_image) {
    if (!gray_image.empty()) {
        vector <double> weights;
        vector <Rect> detected_objects;
        vector <Rect> door_objects;
        vector <double> door_weights;
        hog_.detectMultiScale(
                gray_image,
                detected_objects
                ,weights
                ,0.8         // hit threshold
                ,Size(8,8) // step size
                ,Size(8,8) // padding
                ,1.1       // scale factor
                ,2       // final threshold
                ,false // use mean shift grouping?
                );
        
        for (int i = 0; i < detected_objects.size(); i++) {
            Rect obj = detected_objects.at(i);
            Rect windows(obj.x, obj.y, obj.width, obj.height);
            Mat Roi = gray_image(windows);
            Mat extract, res, train_data;
            resize(Roi, extract, img_size_);
            HogFeatureExtraction(extract);
            ConvertToMl(training_data_, train_data);
            training_data_.clear();
            svm_->predict(train_data, res, 4);
            for (int j = 0; j < res.rows; j++) {
                if (res.at<float>(j, 0) == 1) {
                    door_objects.push_back(obj);
                    door_weights.push_back(weights[j]);
                }
            }
        }
        
        cv::Mat framed_image = color_image.clone();
        DrawLocations(framed_image, door_objects, door_weights, Scalar(0, 255, 0));
        //imshow("view", image);
        //waitKey(1);
        if(door_objects.size() > 0){
            door_count++;
            sensor_msgs::ImagePtr door_img = cv_bridge::CvImage(std_msgs::Header()
                                                       ,"bgr8", color_image).toImageMsg();
            sensor_msgs::ImagePtr framed_img = cv_bridge::CvImage(std_msgs::Header()
                                                       ,"bgr8", framed_image).toImageMsg();
            door_img->header.stamp = framed_img->header.stamp = ros::Time::now();
            //pub_mat_.publish(framed_img);
            
            //publish query message
            lcar_msgs::Door door_query;
            door_query.accepted = false;
            door_query.framed_picture = *framed_img;
            door_query.original_picture = *door_img;
            if(door_count % 5 == 0){
                door_query.query = true;
            }

            pub_query_.publish(door_query);                
        }
    }
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat gray_image = cv_bridge::toCvShare(msg, "mono8")->image;
    //don't change my colorspace! (bgr8))
    cv::Mat color_image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    ObjectCategorize(gray_image, color_image);
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

//double compareHistogram(const cv::Mat& color_mat)
//{
//    Mat hsv_mat;
//    cv::cvtColor(color_mat, hsv_mat, cv::COLOR_BGR2HSV);
//    if(histograms.size() == 0)
//    {
//        histograms.push_back(hsv_mat);
//        return 0.0;
//    }
//    
//    for(const auto& mat: histograms)
//    {
//        double result = cv::compareHist(mat,hsv_mat);
//       // if result > 
//    }
//}

////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv){
  ros::init(argc, argv, "object_detection" /*, ros::init_options::AnonymousName*/ );

  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);

  std::string svmPath = ros::package::getPath("machine_vision");
  svm_ = StatModel::load<SVM>( svmPath + "/LinearHOG.xml" );

  GetSvmDetector(svm_, hog_detector_);

  hog_.winSize = Size(128, 256);
  hog_.setSVMDetector(hog_detector_);

  //pub_mat_ = nh.advertise<sensor_msgs::Image>("object_detection/access_point/door", 1);
  pub_query_ = nh.advertise<lcar_msgs::Door>("object_detection/access_point/door", 1);
  
  std::string ns = ros::this_node::getNamespace();
  std::string topic = "stereo_cam/left/image_rect";
  
  image_transport::Subscriber sub = it.subscribe(topic, 1, imageCallback);
  
  ROS_INFO_STREAM("node: " << ros::this_node::getName() << 
                  " is subscribed to topic: " << ns + "/" + topic);
  
  ros::spin();

  //cv::destroyWindow("view");
}
