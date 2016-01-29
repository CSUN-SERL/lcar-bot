#ifndef MACHINE_LEARNING_H
#define	MACHINE_LEARNING_H

#include <fstream>
#include <thread>
#include <dirent.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <sys/dir.h>
#include <sys/types.h>
#include <queue>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/ml.hpp"
#include "opencv2/objdetect.hpp"

using namespace std;
using namespace cv;
using namespace cv::ml;

class MachineLearning
{
public:
    MachineLearning();
    ~MachineLearning();
    /**
       Traverse file directories.

       @param value string Pass path to file directory.
     */
    void TraverseDirectory(string path);
    /**
       Take an image from the desired folder and run it through image
       processing algorithm.


       @param path
       @param file
       @return
     */
    Mat ProcessImage(string path, string file);
    /**
       Once the image is processed extracted the key features from the image.


       @param ImgMat
       @param imgName
     */
    void Testing(string svm_file);
    /*
     Used to detect objects in images.


      @param svm
     */
    //void HogObjectDetection(Ptr<SVM> svm);
     /*
     Implement Hog feature extraction.


     @param ImgMat
     */
    void HogFeatureExtraction(Mat ImgMat, int label);
    /*
     Implement Orb Feature Extraction

     @param ImgMat
     */
    void OrbFeatureExtraction(Mat ImgMat);
    /*
     breakdown and store a Support Vector Machine object to a a float vector
     that the setSVMDetector is able toe read.

     @param svm
     @param hog_detector
     */
    void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector);
    /*
     Draw rectangles around identified objects.

     @param img
     @param locations
     @param color
     */
    void draw_locations(Mat & img, const vector< Rect > & locations, const Scalar & color, string label);
    /*
     After feature extraction and labeling use autotrain to create a SVM

     @param ml
     */
    void train_svm();
    /*
     Store the vector Mat into a single mat that will be used to train the SVM.

     @param train_samples
     @param trainData
     */
    void convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData);
    /*
     Allow user to choose what type of kernel they want to use for SVM training.

     @param svm
     @param flag
     */
    void set_kernal(Ptr<SVM> svm, int flag);
};
#endif
