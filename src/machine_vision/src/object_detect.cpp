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
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>

#include <iostream>
#include <map>
#include <vector>
#include <string>
#include <lcar_msgs/Query.h>
 
using namespace cv;
using namespace cv::ml;

ros::Publisher pub_mat_;
ros::Publisher pub_query_;

struct ObjectDetectionSubscribers
{
    ros::Subscriber sub_hit_thresh;
    ros::Subscriber sub_step_size;
    ros::Subscriber sub_padding;
    ros::Subscriber sub_scale_factor;
    ros::Subscriber sub_mean_shift;
} od_subs;

ros::Publisher pub_od_request;

struct ObjectDetectionParams
{
  double hit_thresh = 0.45;
  int step_size = 8;
  int padding = 4;
  double scale_factor = 1.15;
  bool mean_shift = false;
} od_params;

Ptr<SVM> svm_;
std::vector< float > hog_detector_;
HOGDescriptor hog_; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9)

//cv::Mat src;
std::vector< Mat > training_data_;
std::vector< cv::Mat > img_array_;
const Size & img_size_ = Size(128, 256);

int door_count = 0;
int img_iterator = 0;

bool gray_scale = false;

static std::string object_types_ [] = {"door", "window", "hole"};
std::map< std::string, std::vector<Mat> > hist_map_;
      /*eg.    "door", <vector of door images>
       *     "window", <vector of window images>
       *     ... TODO: handle other access point types
       */
      
bool compareHistograms(cv::Mat&, std::string);

/////////////////////////////////////////////////////////////////////////////

void HogFeatureExtraction(Mat ImgMat) {
    HOGDescriptor d;
    d.winSize = img_size_;
    std::vector< Point > location;
    std::vector< float > descriptors;
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
    std::vector< Mat >::const_iterator itr = train_samples.begin();
    std::vector< Mat >::const_iterator end = train_samples.end();
    for (int i = 0; itr != end; ++itr, ++i) {
        CV_Assert(itr->cols == 1 || itr->rows == 1);
        if (itr->cols == 1) {
            transpose(*(itr), tmp);
            tmp.copyTo(trainData.row(i));
        } else if (itr->rows == 1) {
            itr->copyTo(trainData.row(i));
        }
    }
}

//Draw detected objects on a window
void DrawLocations(cv::Mat& img, const std::vector< Rect > & found, const Scalar & color) {
       for (auto const& loc : found) {
               //Rect implementation used with hog.detectMultiScale
               rectangle(img, loc, color, 2); 
       }
}

void ObjectCategorize(const cv::Mat& gray_image, cv::Mat& color_image) {
    if (!gray_image.empty()) {
        std::vector <double> weights;
        std::vector <Rect> detected_objects;
        std::vector <Rect> door_objects;
        std::vector <double> door_weights;
        hog_.detectMultiScale(
                gray_image,
                detected_objects
                ,weights
                ,od_params.hit_thresh                           // hit threshold
                ,Size(od_params.step_size, od_params.step_size) // step size
                ,Size(od_params.padding, od_params.padding)     // padding
                ,od_params.scale_factor                         // scale factor
                ,2                                            // final threshold
                ,od_params.mean_shift                // use mean shift grouping?
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
        DrawLocations(framed_image, door_objects, Scalar(0, 255, 0));
        //imshow("view", image);
        //waitKey(1);
        if(door_objects.size() > 0){
            sensor_msgs::ImagePtr door_img = cv_bridge::CvImage(std_msgs::Header()
                                                       ,"bgr8", color_image).toImageMsg();
            sensor_msgs::ImagePtr framed_img = cv_bridge::CvImage(std_msgs::Header()
                                                       ,"bgr8", framed_image).toImageMsg();
            door_img->header.stamp = framed_img->header.stamp = ros::Time::now();
            door_img->header.seq = framed_img->header.seq = door_count;
            //pub_mat_.publish(framed_img);
            
            //publish query message
            lcar_msgs::Query door_query;
            door_query.is_accepted = false;
            door_query.img_framed = *framed_img;
            door_query.img = *door_img;

            bool similar = compareHistograms(color_image, "door");
            if(!similar)
            {
                pub_query_.publish(door_query);         
                door_count++;
            }
        }
    }
}

bool compareHistograms(cv::Mat& src, std::string object_type)
{   
    ROS_INFO_STREAM("////////////////");
    
    cv::Mat hist;
    if(!gray_scale)
    {
        cv::Mat hsv;
        src.convertTo(hsv, CV_32F);
        cv::cvtColor(hsv, hsv, cv::COLOR_BGR2HSV);
        // Quantize the hue to 30 levels
        // and the saturation to 32 levels
        int hbins = 60, sbins = 64;
        int histSize[] = {hbins, sbins};
        // hue varies from 0 to 179, see cvtColor
        float hranges[] = { 0, 180 };
        // saturation varies from 0 (black-gray-white) to
        // 255 (pure spectrum color)
        float sranges[] = { 0, 256 };
        const float* ranges[] = { hranges, sranges };
        // we compute the histogram from the 0-th and 1-st channels
        int channels[] = {0, 1};
        
        cv::calcHist( &hsv, 1, channels, Mat(), // do not use mask
                 hist, 2, histSize, ranges,
                 true, // the histogram is uniform
                 false );
    }
    else
    {
        cv::Mat mat = src.clone();
        cv::cvtColor(mat, mat, cv::COLOR_BGR2GRAY);

        int histSize = 32;    // bin size
        float range[] = { 0, 256 };
        const float *ranges[] = { range };
        
        cv::calcHist( &mat, 1, 0, Mat(), 
                      hist, 1, &histSize, 
                      ranges, true, false );
    }
    
    normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
    
    std::vector<Mat>* stream = &hist_map_[object_type];
    ROS_WARN_STREAM("size: " << stream->size());
    for(int i = 0; i < stream->size(); i++)
    {
        double result = cv::compareHist(hist, stream->at(i), CV_COMP_CHISQR);
        //ROS_WARN_STREAM(result);
        if(result < 2.5)
            return true; 
    }
    
    stream->push_back(hist);
    return false;
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
//    if(msg->encoding == "mono8")
//        gray_scale = true;
      
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
void GetSvmDetector(const Ptr<SVM>& svm, std::vector< float > & hog_detector) {
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
///

void hitThresholdCallback(const std_msgs::Float64& msg)
{
    od_params.hit_thresh = msg.data;
    ROS_INFO_STREAM("received new hit threshold: " << msg.data);
}

void stepSizeCallback(const std_msgs::Int32& msg)
{
    od_params.step_size = msg.data;
    ROS_INFO_STREAM("received new step size: " << msg.data);
}

void paddingCallback(const std_msgs::Int32& msg)
{
    od_params.padding = msg.data;
    ROS_INFO_STREAM("received new padding: " << msg.data);
}

void scaleFactorCallback(const std_msgs::Float64& msg)
{
    od_params.scale_factor = msg.data;
    ROS_INFO_STREAM("received scale factor: " << msg.data);
}

void meanShiftCallback(const std_msgs::Int32& msg)
{
    od_params.mean_shift = msg.data;
    ROS_INFO_STREAM("received mean shift value: " << msg.data);
}

////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv){
  ros::init(argc, argv, "object_detection" /*, ros::init_options::AnonymousName*/ );

  ros::NodeHandle nh;
  //cv::namedWindow("view");
  //cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  
  std::string svmPath = ros::package::getPath("machine_vision");
  svm_ = StatModel::load<SVM>( svmPath + "/LinearHOG.xml");

  GetSvmDetector(svm_, hog_detector_);

  hog_.winSize = Size(128, 256);
  hog_.setSVMDetector(hog_detector_);

  //pub_mat_ = nh.advertise<sensor_msgs::Image>("object_detection/access_point/door", 1);
  pub_query_ = nh.advertise<lcar_msgs::Query>("object_detection/access_point/door", 1);
  
  std::string ns = ros::this_node::getNamespace();
  std::string topic = "stereo_cam/left/image_rect";
  
  image_transport::Subscriber sub = it.subscribe(topic, 2, imageCallback);
  
  od_subs.sub_hit_thresh = nh.subscribe("/object_detection/hit_threshold", 5, hitThresholdCallback);
  od_subs.sub_step_size = nh.subscribe("/object_detection/step_size", 5, stepSizeCallback);
  od_subs.sub_padding = nh.subscribe("/object_detection/padding", 5, paddingCallback);
  od_subs.sub_scale_factor = nh.subscribe("/object_detection/scale_factor", 5, scaleFactorCallback);
  od_subs.sub_mean_shift = nh.subscribe("/object_detection/mean_shift_grouping", 5, meanShiftCallback);
  
  pub_od_request = nh.advertise<std_msgs::Int32>("/object_detection/param_request", 2);
  pub_od_request.publish(std_msgs::Int32());
  
  
  ROS_INFO_STREAM("node: " << ros::this_node::getName() << 
                  " is subscribed to topic: " << ns + "/" + topic);
  
  ros::spin();

  //cv::destroyWindow("view");
}
