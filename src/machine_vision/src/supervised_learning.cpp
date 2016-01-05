#include <opencv2/objdetect.hpp>

#include "machine_learning.h"
#include "boost/algorithm/string.hpp"

using namespace boost;
//#include <sstream>

void set_label(cv::Mat& im, cv::Rect r, const std::string label);
void test_Hog();
void sample_neg(Mat greyimg);

Mat labels;
vector<Mat> trainingdata;
Mat src;
Mat display;
queue<string> proc_img;
int testing;
int N, size_;
float doorcounter, negcounter;
int number = 0;
int label_ = 0;
int object = -1;
string user = getenv("USER");
Ptr<SVM> svm;


void OrbDetection(Ptr<SVM> svm);

int main(int argc, char * argv[]) {
    MachineLearning ml;
    //ml.train_svm();
    const string modelLibPath = "SVM.xml";
    svm = StatModel::load<SVM>(modelLibPath);

    //    //Ptr<SVM> Svm = SVM::create();
    //trainingdata.release();
    //ml.Testing(ml);
    //ml.Hog(svm);
    //OrbDetection(svm);
    test_Hog();
    labels.release();
    src.release();

    return 0;
}

MachineLearning::MachineLearning(void) {//Class constructor

}

MachineLearning::~MachineLearning(void) {
    //Class destructor
}

void MachineLearning::TraverseDirectory(string path) {
    DIR * tree;

    if ((tree = opendir(&path[0])) == NULL) {
        perror(string("ERROR: " + path).c_str());
        exit(1);
    }

    struct dirent * dirEntry;
    string newPath;
    while ((dirEntry = readdir(tree)) != NULL) {
        if (dirEntry->d_ino == 0)
            continue;

        if (dirEntry->d_type == DT_DIR && dirEntry->d_name[0] != '.') {
            newPath = path + "/" + dirEntry->d_name;
            cout << "\n\nEntering: " << newPath << endl;
            /*if (contains(newPath, "pos")) {
                label_ = +1;
                cout << label_ << "\n";
            }
            if (contains(newPath, "neg")) {
                label_ = -1;
                cout << label_ << "\n";
            }*/
            TraverseDirectory(newPath);
            //cout << N << "\n";
            label_ = N++;
        } else if (dirEntry->d_type == DT_REG && dirEntry->d_name[0] != '.') {

            Mat greyImgMat = MachineLearning::ProcessImage(path, dirEntry->d_name);
            if (testing == 2) {
                sample_neg(greyImgMat);
            }
            //MachineLearning::ExtractFeatures(greyImgMat, dirEntry->d_name);
            MachineLearning::HogFeatureExtraction(greyImgMat, label_);

        }
    }

    closedir(tree);
}

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

    resize(src, src, Size(64, 128));

    //imshow(file, src);
    //waitKey(0);

    //blur image to reduce number of features

    GaussianBlur(src, src, Size(9, 9), 2, 2);
    //erode image
/*
    Mat kernel = Mat::ones(3, 3, CV_8U);
    Mat eroded;
    erode(src, eroded, kernel);
    src = src - eroded;
*/
    //convert image to grayscale
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    //Apply Otsu thresholding
    //threshold(gray, gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);


    //dilate(gray, gray, Mat(), Point(-1, -1), 2, 1, 1);


    return gray;

}

void MachineLearning::ExtractFeatures(Mat ImgMat, string imgName) {
    ////////////////////////////////////////////// migrated from processFeatures

    //apply canny edge detector
    Mat bw;
    cv::Canny(ImgMat, bw, 0, 50, 5);

    //Find Houghlines
    std::vector<cv::Vec4i> lines_vec;
    HoughLinesP(bw, lines_vec, 1, CV_PI / 180, 100, 30, 10);

    //cv::imshow("8", bw);

    for (size_t i = 0; i < lines_vec.size(); i++) {
        line(src, Point(lines_vec[i][0], lines_vec[i][1]), Point(lines_vec[i][2], lines_vec[i][3]), Scalar(0, 0, 255), 3, 8);
        //cv::imshow("9", src);
    }

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    //cv::imshow("9", contours);
    Mat drawing = Mat::zeros(bw.size(), CV_8UC3);
    Scalar colors[3];
    colors[0] = Scalar(255, 0, 0);
    colors[1] = Scalar(0, 255, 0);
    colors[2] = Scalar(0, 0, 255);
    for (int i = 0; i < contours.size(); i++) {
        drawContours(src, contours, i, colors[i % 3]);
        // cv::imshow("10", src);
    }

    ////////////////////////////////////////////////////////////////////////////

    Ptr<ORB> detector = ORB::create(500, 1.2, 3, 15, 0);
    vector<KeyPoint> keypoints_1;

    //-- Draw keypoints
    cv::Mat img_keypoints_1;
    //detect keypoints
    detector->detect(src, keypoints_1);

    //describe keypoints
    detector->compute(src, keypoints_1, img_keypoints_1);
    //copy.copyTo(img_keypoints_1);
    drawKeypoints(src, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    //imshow("c", img_keypoints_1);

    //imshow(imgName, img_keypoints_1);
    //waitKey();

    if (!img_keypoints_1.empty()) {
        trainingdata.push_back(img_keypoints_1.reshape(1, 1));
        if (imgName != " ")
            labels.push_back(N);
        size_++;
    }
}

void MachineLearning::Testing() {
    testing = 1;
    const string modelLibPath = "SVM.xml";
    //Ptr<SVM> Svm = SVM::create();
    Ptr<SVM> Svm = StatModel::load<SVM>(modelLibPath);
    //ml.IMAGES_DIR = "/home/" + user + "/Desktop/Doors";
    string IMAGES_DIR = "/home/" + user + "/Pictures/Training/neg";
    TraverseDirectory(IMAGES_DIR);

    //trainingdata.convertTo(trainingdata, CV_32FC1);
    bool c = Svm->isClassifier();
    bool t = Svm->isTrained();
    cout << c << " " << t << "\n";
    Mat train_data;
    convert_to_ml(trainingdata, train_data);
    Mat res;
    Svm->predict(train_data, res, 4);


    for (int i = 0; i < res.rows; i++) {
        if (res.at<float>(i, 0) == 0)
            doorcounter++;
        else
            negcounter++;
        cout << res.row(i) << " " << proc_img.front() << "\n"; // << "Prediction: " << p << "\n";
        proc_img.pop();
    }
    float doors = doorcounter / res.rows;
    cout << "Number of doors: " << doorcounter << "/" << res.rows << "\n";
    cout << "Number of other objects: " << negcounter << "/" << res.rows << "\n";
    cout << "Percent doors: " << doors * 100.0 << "%\n";
}

void MachineLearning::Hog(Ptr<SVM> svm) {
    MachineLearning ml;
    VideoCapture cap;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    Scalar window(255, 0, 0);
    vector< Rect > doors;
    get_svm_detector(svm, hog_detector);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    //        if (!cap.isOpened())
    //            return -1;
    Mat img, draw;
    HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.winSize = Size(64, 128);
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

        //Mat greyimg = ml.ProcessImage("", "");
        //ml.HogFeatureExtraction(greyimg, -1);
        //Mat res, predict;
        //ml.convert_to_ml(trainingdata, predict);
        //trainingdata.convertTo(trainingdata, CV_32FC1);
        //float p = svm->predict(predict, res, 4);
        //cout << "Prediction: " << p << "\n";
        //for (int i = 0; i < res.rows; i++) {
        //if (res.at<float>(i, 0) == 0){

        draw = img.clone();

        doors.clear();
        hog.detectMultiScale(img, doors, true);
        //people.detectMultiScale(img, locations);
        MachineLearning::draw_locations(draw, doors, reference, "door");
        //cout << "door\n";
        imshow("video capture", draw);
        //trainingdata.release();
        // }
        //else{
        //imshow("video capture", img);
        //}
        //}


        if (waitKey(10) >= 0)
            break;
    }
}

void MachineLearning::HogFeatureExtraction(Mat ImgMat, int label) {

    HOGDescriptor d;
    d.winSize = Size(64, 128);
    vector< Point > location;
    vector< float > descriptors;
    d.compute(ImgMat, descriptors, Size(8, 8), Size(0, 0), location);
    trainingdata.push_back(Mat(descriptors).clone());
    if (label != -1)
        labels.push_back(N); //labels.push_back(label);
    //cout << label << "\n";
    size_++;

}

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

void MachineLearning::draw_locations(Mat & img, const vector< Rect > & found, const Scalar & color, string label) {
    if (!found.empty()) {
        vector<Rect> found_filtered;
        size_t i, j;
        for (i = 0; i < found.size(); i++) {
            Rect r = found[i];
            for (j = 0; j < found.size(); j++)
                if (j != i && (r & found[j]) == r)
                    break;
            if (j == found.size())
                found_filtered.push_back(r);
        }
        for (i = 0; i < found_filtered.size(); i++) {
            Rect r = found_filtered[i];
            r.x += cvRound(r.width * 0.1);
            r.width = cvRound(r.width * 0.8);
            r.y += cvRound(r.height * 0.06);
            r.height = cvRound(r.height * 0.9);
            rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
            set_label(img, r, label);
        }
        /*
                vector< Rect >::const_iterator loc = found.begin();
                vector< Rect >::const_iterator end = found.end();
                for (; loc != end; ++loc) {
                    rectangle(img, *loc, color, 2);
                    set_label(img, *loc, label);
                }
         */

    }
}

void sample_neg(Mat greyimg) {
    MachineLearning ml;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    vector< Rect > locations;
    ml.get_svm_detector(svm, hog_detector);
    //        if (!cap.isOpened())
    //            return -1;
    Mat draw, img;
    HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.winSize = Size(64, 128);
    hog.setSVMDetector(hog_detector);
    src = greyimg;
    resize(src, src, Size(256, 512));
    resize(display, display, Size(256, 512));
    // Parameters of your slideing window
    int windows_n_rows = 60;
    int windows_n_cols = 60;
    // Step of each window
    int StepSlide = 30;

    // Just copy of Loaded image
    // Note that Mat img = LoadedImage; This syntax only put reference on LoadedImage
    // Whot does it mean ? if you change img, LoadeImage is changed 2.
    // IF you want to make a copy, and do not change the source image- Use clone();
    Mat DrawResultGrid = display.clone();

    // Cycle row step
    //for (int row = 0; row <= src.rows - windows_n_rows; row += StepSlide) {
        // Cycle col step
       // for (int col = 0; col <= src.cols - windows_n_cols; col += StepSlide) {
            // There could be feature evaluator  over Windows

            // resulting window
            //Rect windows(col, row, windows_n_rows, windows_n_cols);
            //resize(img, img, Size(64, 128));

            locations.clear();
            hog.detectMultiScale(src, locations, true);
            for(int i = 0; i < locations.size();i++){
            Rect windows(locations.at(i).x, locations.at(i).y, locations.at(i).width, locations.at(i).height);
            img = display(windows).clone();
            Mat Roi = display(windows);

            //Show ROI
            namedWindow("Step 4 Draw selected Roi", WINDOW_AUTOSIZE);
            imshow("Step 4 Draw selected Roi", Roi);
            waitKey(100);
            imwrite("Step4.JPG", Roi);
            }
            Mat DrawResultHere = display.clone();

            // Draw only rectangle
            if (!locations.empty()) {
                vector< Rect >::const_iterator loc = locations.begin();
                vector< Rect >::const_iterator end = locations.end();
                for (; loc != end; ++loc) {
                    rectangle(DrawResultHere, *loc, Scalar(255), 1,8,0);
                    // Show  rectangle
            namedWindow("Step 2 draw Rectangle", WINDOW_AUTOSIZE);
            imshow("Step 2 draw Rectangle", DrawResultHere);
            waitKey(100);
            imwrite("Step2.JPG", DrawResultHere);

                }
            }
            //rectangle(DrawResultHere, windows, Scalar(255), 1, 8, 0);
            //ml.draw_locations(DrawResultHere, locations, reference, "door");
            // Draw grid
            //rectangle(DrawResultGrid, windows, Scalar(255), 1, 8, 0);


            // Show  rectangle
            namedWindow("Step 2 draw Rectangle", WINDOW_AUTOSIZE);
            imshow("Step 2 draw Rectangle", DrawResultHere);
            waitKey(100);
            imwrite("Step2.JPG", DrawResultHere);

            // Show grid
/*
             if (!locations.empty()) {
                vector< Rect >::const_iterator loc = locations.begin();
                vector< Rect >::const_iterator end = locations.end();
                for (; loc != end; ++loc) {
                    rectangle(DrawResultGrid, *loc, Scalar(255), 1,8,0);
                    // Show  rectangle


            namedWindow("Step 3 Show Grid", WINDOW_AUTOSIZE);
            imshow("Step 3 Show Grid", DrawResultGrid);
            waitKey(100);
            imwrite("Step3.JPG", DrawResultGrid);
                }}
*/
            // Select windows roi
/*
            Mat Roi = src(windows);

            //Show ROI
            namedWindow("Step 4 Draw selected Roi", WINDOW_AUTOSIZE);
            imshow("Step 4 Draw selected Roi", Roi);
            waitKey(100);
            imwrite("Step4.JPG", Roi);
*/
       // }
   // }



}

void OrbDetection(Ptr<SVM> svm) {
    VideoCapture cap(-1);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1000);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1000);
    //    if (!cap.isOpened())
    //        return -1;
    MachineLearning ml;

    Mat img;
    vector<KeyPoint> keypoints_1;
    Ptr<ORB> detector = ORB::create(500, 1.2, 3, 15, 0);
    while (true) {

        cap >> img;
        if (!img.data)
            continue;
        src = img;
        Mat greyimg = ml.ProcessImage(" ", " ");
        ml.ExtractFeatures(greyimg, " ");
        //ml.HogFeatureExtraction(greyimg);
        //        cvtColor(img, img, CV_BGR2GRAY);
        //
        //        vector<Rect> found, found_filtered;
        cv::Mat img_keypoints_1, res;
        //        //MachineLearning::ExtractFeatures(img, "");
        //
        //        //detect keypoints
        detector->detect(img, keypoints_1);
        //        //describe keypoints
        detector->compute(img, keypoints_1, img_keypoints_1);
        std::vector<Point2f> obj_corners(4);
        drawKeypoints(img, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        imshow("video capture", greyimg);
        //        resize(img_keypoints_1, img_keypoints_1, Size(640, 480));
        //
        //        img_keypoints_1.convertTo(img_keypoints_1, CV_32FC1);
        //        img_keypoints_1.reshape(1, 1);
        //trainingdata.convertTo(trainingdata, CV_32FC1);
        float p = svm->predict(trainingdata, res, 4);
        //
        //
        for (int i = 0; i < res.rows; i++) {
            if (res.at<float>(i, 0) == 0 && object != 0) {
                object = 0;
                cout << "Door detected\n";
            }
            if (res.at<float>(i, 0) == 1 && object != 1) {
                object = 1;
                cout << "No doors detected\n";
            }
            //trainingdata.release();
        }
        if (waitKey(20) >= 0)
            break;

    }
}

void MachineLearning::set_kernal(Ptr<SVM> svm, int flag) {
    switch (flag) {
        case 1: svm->setKernel(SVM::LINEAR);
            cout << "Linear\n";
            break;
        case 2: svm->setKernel(SVM::RBF);
            cout << "RBF\n";
            break;
        case 3: svm->setKernel(SVM::CHI2);
            cout << "CHI\n";
            break;
        case 4: svm->setKernel(SVM::POLY);
            svm->setDegree(2);
            cout << "Poly\n";
            break;
        case 5: svm->setKernel(SVM::SIGMOID);
            cout << "SIGMOID\n";
            break;

    }
}

void MachineLearning::train_svm() {
    MachineLearning ml;
    string IMAGES_DIR = "/home/" + user + "/Pictures/Training";

    ml.TraverseDirectory(IMAGES_DIR);

    labels.convertTo(labels, CV_32SC1);
    //trainingdata.convertTo(trainingdata, CV_32FC1);
    Mat train_data;
    convert_to_ml(trainingdata, train_data);
    //Set up SVM's parameters
    Ptr<SVM> svm = SVM::create();
    svm->setKernel(SVM::LINEAR);
    //ml.set_kernal(svm, 2);
    Ptr<TrainData> td = TrainData::create(train_data, ROW_SAMPLE, labels);

    cout << "inside main training\n";
    svm->trainAuto(td, 20);
    svm->save("SVM.xml");
    //trainingdata.release();

}

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

void test_Hog() {
    MachineLearning ml;
    const string modelLibPath = "SVM.xml";
    svm = StatModel::load<SVM>(modelLibPath);

    string IMAGES_DIR = "/home/" + user + "/Pictures/Training/doors";
    testing = 2;
    ml.TraverseDirectory(IMAGES_DIR);


}
