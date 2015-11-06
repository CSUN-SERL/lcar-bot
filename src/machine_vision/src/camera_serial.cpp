#include "opencv2/opencv.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{

    Mat frameLeft;
    Mat frameRight;

    VideoCapture lCap(0);
    VideoCapture rCap(1);

  //  namedWindow("cam_left",  WINDOW_KEEPRATIO);
    //namedWindow("cam_right", WINDOW_KEEPRATIO);

    if(!lCap.isOpened() || !rCap.isOpened()) return -1;

    lCap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    lCap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    rCap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    rCap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    for(;;){
        lCap >> frameLeft;
        rCap >> frameRight;

        if(!frameLeft.empty())
          imshow("cam_left", frameLeft);

        if(!frameRight.empty())
          imshow("cam_right", frameRight);

        if(waitKey(30) >= 0) break;

    }

    return 0;
}
