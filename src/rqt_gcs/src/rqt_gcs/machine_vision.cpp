#include "machine_learning.h"

//#include <sstream>

void set_label(cv::Mat& im, cv::Rect r, const std::string label);
void test_Hog();
void sample_neg(Mat greyimg);
void catergorize();
void HogObjectDetection(Ptr<SVM> svm);
void run();
void create_svm();
void test_svm();
void detect_objects();
void static_HOGtest();
void detect_object(Mat img);
void reset_globals();

//Globals for object catergorization
bool running = false;
queue<Mat> detected_objects;
queue<Mat> categorized_objects;
queue<int> categorized_labels;

//Globals for user control
vector<int> svm_kernels;
queue<int> feature_extraction;

//Globals mats used for training
Mat labels;
vector<Mat> trainingdata;
Size img_size = Size(64, 128);

//Copys of current image
Mat src; //original
Mat display; //used for displaying purposes

//Global queues used to track img names and labels for testing
queue<string> proc_img;
queue<int> img_label;

//Parameter used to turn testing on or off
int testing;

//N is label tracker, size_ is for total number of images read
int N, size_;

//variables to be used in the testing process
float doorcounter, negcounter, windowcounter;
float static_correct, static_total;

//what the fuck are these
int number = 0;
int label_ = 0;
int neg_count = 2315;
int object = -1;

//Strings that are resued
string user = getenv("USER");
string svm_type; //stores kernel type
string extraction_type;

//May later be used for testing purposes
float best_correct;
string best_svm;


int iteration; //number corresponds to svm
int iteration_2; //number correspons to extraction type

MachineLearning::MachineLearning(void) {//Class constructor

}

MachineLearning::~MachineLearning(void) {//Class destructor

}

//detect_objects() important
//The goal of this method is detect and classify objects in real time
void detect_objects() {
    string modelLibPath = "LinearHOG.xml";
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    thread thread_1(HogObjectDetection, svm);
    thread thread_2(catergorize);

    thread_1.join();
    thread_2.join();
}

//Convert a vector mat into a single mat
void MachineLearning::convert_to_ml(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData) {
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

//extract svm weights and store in a float vector
void MachineLearning::get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector) {
    // get the support vectors
    Mat sv = svm->getSupportVectors();
    const int sv_total = sv.rows;
    // get the decision function
    Mat alpha, svidx;
    double rho = svm->getDecisionFunction(0, alpha, svidx);
    cout << alpha.total();
    cout << "\n";
    svidx.reshape(1, 1);
    cout << svidx.total();
    cout << "\n";
    cout << sv_total;
    cout << "\n";
    cout << rho << "\n";
    //    CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
    //    CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
    //            (alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
    //    CV_Assert(sv.type() == CV_32F);
    hog_detector.clear();

    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof (hog_detector[0]));
    hog_detector[sv.cols] = (float) -rho;
}

//Resize and grayscale, maybe add some more processing techinques later on
Mat MachineLearning::ProcessImage(string path, string file) {
    if (path != "" && file != "") {
        //cout << "grey scaling: " << file << endl;
        src = imread(path + "/" + file);
        display = src.clone();
        if (testing == 1) {
            proc_img.push(file);
        }
    }
    if (!src.data) {
        cout << "no image" << endl;
        exit(0);
    }

    resize(src, src, img_size);
    //convert image to grayscale
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    //threshold(gray, gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
    //cv::Canny(gray, gray, 0, 50, 5);
    src.release();
    return gray;

}

//Feature extraction based on HOG
void MachineLearning::HogFeatureExtraction(Mat ImgMat, int label) {
    HOGDescriptor d;
    d.winSize = img_size;
    vector< Point > location;
    vector< float > descriptors;
    d.compute(ImgMat, descriptors, Size(8, 8), Size(0, 0), location);
    trainingdata.push_back(Mat(descriptors).clone());
    if (label != -1)
        labels.push_back(label); //labels.push_back(label);
    //cout << label << "\n";
    if (testing == 1) {
        img_label.push(label);
    }
    size_++;
    extraction_type = "HOG";

}

//Attatch a  label to a detected object
void set_label(cv::Mat& im, cv::Rect r, const std::string label) {
    int fontface = cv::FONT_HERSHEY_SIMPLEX;
    double scale = 0.5;
    int thickness = 1;
    int baseline = 0;

    cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
    cv::Point pt(r.x + (r.width - text.width) / 2, r.y + (r.height + text.height) / 2);

    cv::rectangle(
            im,
            pt + cv::Point(0, baseline),
            pt + cv::Point(text.width, -text.height),
            CV_RGB(255, 0, 0), CV_FILLED
            );

    cv::putText(im, label, pt, fontface, scale, CV_RGB(255, 255, 0), thickness, 8);
}

//Perform real time HOG object detection
void HogObjectDetection(Ptr<SVM> svm) {
    running = true;
    MachineLearning ml;
    VideoCapture cap;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    Scalar window(255, 0, 0);
    vector< Rect > objects;
    ml.get_svm_detector(svm, hog_detector);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    //        if (!cap.isOpened())
    //            return -1;
    Mat img, draw;
    HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.winSize = img_size;

    //HOGDescriptor people;
    //hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    hog.setSVMDetector(hog_detector);

    //int value = 0;
    namedWindow("video capture", CV_WINDOW_AUTOSIZE);
    cap.open(0);
    while (true) {
        cap >> img;
        if (!img.data)
            break;
        src = img;
        objects.clear();
        hog.detectMultiScale(img, objects, true);
        //people.detectMultiScale(img, locations);
        ml.draw_locations(img, objects, reference, "door");
        //cout << "door\n";
        imshow("video capture", img);

        if (waitKey(10) >= 0)
            break;
    }
}

//While objects are being detected catergorize them and store the ones that are desired object
void catergorize() {
    const string modelLibPath = "LinearHOG.xml";
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    MachineLearning ml;
    while (true) {
        if (!detected_objects.empty()) {
            Mat object, train_data, res;
            resize(detected_objects.front(), object, img_size);
            detected_objects.pop();
            ml.HogFeatureExtraction(object, -1);
            ml.convert_to_ml(trainingdata, train_data);
            trainingdata.clear();
            svm->predict(train_data, res, 4);
            for (int j = 0; j < res.rows; j++) {
                cout << res.at<float>(j, 0) << " " << "res " << res.rows << "\n";
            }
        }
    }
}

//Draw detected objects on a window
void MachineLearning::draw_locations(Mat & img, const vector< Rect > & found, const Scalar & color, string label) {
    if (!found.empty()) {
      vector< Rect >::const_iterator loc = found.begin();
      vector< Rect >::const_iterator end = found.end();
      for (; loc != end; ++loc) {
          rectangle(img, *loc, color, 2);
          set_label(img, *loc, label);
          detected_objects.push(img.clone());
      }
    }
}

void reset_globals(){
    labels.release();
    src.release();
    trainingdata.clear();
}
