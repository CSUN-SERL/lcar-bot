#include "opencv2/opencv.hpp"

#include <iostream>
#include <tbb/parallel_invoke.h>

using namespace cv;
using namespace cv::ml;
using namespace std;

int main(int argc, char** args) {

//    cv::Mat frameLeft;
//    cv::Mat frameRight;

    tbb::parallel_invoke(
        [& ]() {
            cv::Mat frameLeft;
            VideoCapture lCap(0);

            if (!lCap.isOpened()) return -1;

            lCap.set(CV_CAP_PROP_FPS, 15);
            lCap.set(CV_CAP_PROP_FRAME_WIDTH , 352);
            lCap.set(CV_CAP_PROP_FRAME_HEIGHT , 255);

            //namedWindow("cam_left", WINDOW_KEEPRATIO);
            for (;;) {
                lCap >> frameLeft;
                if (!frameLeft.empty())
                    imshow("cam_left", frameLeft);

                if (waitKey(30) >= 0) break;
            }
        },
        [& ]() {
            cv::Mat frameRight;
            VideoCapture rCap(1);

            if (!rCap.isOpened()){
                cout << "broken";
                return -1;
            }

            rCap.set(CV_CAP_PROP_FPS, 15);
            rCap.set(CV_CAP_PROP_FRAME_WIDTH , 352);
            rCap.set(CV_CAP_PROP_FRAME_HEIGHT , 255);

            for (;;) {
                rCap >> frameRight;
                if (!frameRight.empty())
                    imshow("cam_right", frameRight);

                cout << "right\n";

                if (waitKey(30) >= 0) break;
            }
        }
    );

    return 0;
}
