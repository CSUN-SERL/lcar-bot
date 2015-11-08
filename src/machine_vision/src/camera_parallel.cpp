#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "opencv2/opencv.hpp"


using namespace cv;
using namespace cv::ml;
using namespace std;

///*
int main (int argc, char ** argv){

	int pid = fork();

	if(pid > 0){
	//for	(int i = 0; i < 1000; i++);
		///*
        VideoCapture lCap(0);

        if (!lCap.isOpened()) return -1;
		
		lCap.set(CV_CAP_PROP_FPS, 5);
        lCap.set(CV_CAP_PROP_FRAME_WIDTH , 640);
        lCap.set(CV_CAP_PROP_FRAME_HEIGHT , 480);

        namedWindow("cam_left", CV_WINDOW_AUTOSIZE);
        cv::Mat frameLeft;
        for (;;) {
            ///*
            lCap >> frameLeft;
            if (!frameLeft.empty())
                imshow("cam_left", frameLeft);
		
            if (waitKey(5) >= 0) break;
            //
            //cout << "left\n";
        }
        lCap.release();	
        //*/
	}
	else if(pid == 0){
		//while(true){
		try{
        VideoCapture rCap(1);

        if (!rCap.isOpened()) return -1;
        
		rCap.set(CV_CAP_PROP_FPS, 5);
        rCap.set(CV_CAP_PROP_FRAME_WIDTH , 640);
        rCap.set(CV_CAP_PROP_FRAME_HEIGHT , 480);

	    namedWindow("cam_right", CV_WINDOW_AUTOSIZE);
	    cv::Mat frameRight;
        for (;;) {
            ///*
            rCap >> frameRight;
            if (!frameRight.empty())
                imshow("cam_right", frameRight); 

            if (waitKey(5) >= 0) break;
            //*/
            cout << "right\n";
        }
        rCap.release();
        }
        catch(cv::Exception& ex){ }
        //}
	}
	else{
		cout << "error forking";
		exit(1);
	}

return 0;
}

//*/
