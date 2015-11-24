#ifndef MACHINE_LEARNING_H
#define	MACHINE_LEARNING_H

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
    void ExtractFeatures(Mat ImgMat, string imgName);
    /**
       Used to test accuracy of the model library.


       @param svm
       @param ml
     */
    void Testing(Ptr<SVM> svm, MachineLearning ml);

    string IMAGES_DIR;

};

#endif
