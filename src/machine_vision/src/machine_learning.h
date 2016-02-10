#ifndef MACHINE_LEARNING_H
#define	MACHINE_LEARNING_H

#include <sys/dir.h>
#include <sys/types.h>
#include <stdio.h>
#include <dirent.h>

#include <fstream>
#include <thread>
#include <iostream>
#include <string>
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
     Given a path to a folder this will traverse all subdirectories
     and files.

       @param path
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
    /*
     Passed the path of a svm file, this will perform a classification test.

      @param svm_file
     */
    void Testing(string svm_file);
    /*
     Used to detect objects in images.


      @param svm
     */
    //void HogObjectDetection(Ptr<SVM> svm);
     /**
       Once the image is processed extracted the key features from the image,
      * using HOG feature extraction.


       @param ImgMat
       @param label
     */
    void HogFeatureExtraction(Mat ImgMat, int label);

    /*
     breakdown and store a Support Vector Machine object to a a float vector
     that the setSVMDetector is able toe read.

     @param svm
     @param hog_detector
     */
    void GetSvmDetector(const Ptr<SVM>& svm, vector< float > & hog_detector);
    /*
     Draw rectangles around identified objects.

     @param img
     @param locations
     @param color
     @param label
     */
    void DrawLocations(Mat & img, const vector< Rect > & locations, const Scalar & color, string label);
    /*
     After feature extraction and labeling use autotrain to create a SVM

     @param ml
     */
    void TrainSvm();
    /*
     Store the vector Mat into a single mat that will be used to train the SVM.

     @param train_samples
     @param trainData
     */
    void ConvertToMl(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData);
    /*
     Allow user to choose what type of kernel they want to use for SVM training.

     @param svm
     @param flag
     */
    void SetKernal(Ptr<SVM> svm, int flag);

    /**EXPERIMENTAL FUNCTIONS*/
    /*
    Implement Orb Feature Extraction

     @param ImgMat
    */
    void OrbFeatureExtraction(Mat ImgMat);
};
#endif //MACHINE_LEARNING_H
