#include <fstream>
#include <thread>

#include "machine_learning.h"
#include "boost/algorithm/string.hpp"

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
Mat orb_trainingdata;

//Copys of current image
Mat src; //original
Mat display; //used for negative mining display purposes

//Global queues used to track img names and labels for testing
queue<string> proc_img;
queue<int> img_label;

//Parameter used to turn testing on or off
int testing;

//N is label tracker, size_ is for total number of images read
int N, size_;

//variables to be used in the testing process
float doorcounter, negcounter, windowcounter;

//what the fuck are these
int number = 0;
int label_ = 0;
int neg_count = 0;
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

int main(int argc, char * argv[]) {
    run();
    //ml.train_svm();
    //    const string modelLibPath = "LinearHOG.xml";
    //    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    //
    //    //    //Ptr<SVM> Svm = SVM::create();
    //    //trainingdata.release();
    //    //ml.Testing("LinearORB.xml");
    //    //test_Hog();
    //    std::thread thread_1(HogObjectDetection, svm);
    //    std::thread thread_2(catergorize);
    //
    //    thread_1.join();
    //    thread_2.join();
    //  thread_1.detach();
    //  thread_2.detach();
    //OrbDetection(svm);
    //test_Hog();
    //   run();
    labels.release();
    src.release();
    return 0;
}

void run() {
    int user_input = 0;
    do {
        cout << "What are you planning on doing?\n";
        cout << "(1)Train a SVM\n(2)Test a current SVM\n(3)Apply negative mining to an existing SVM\n(4)Run object detection\n(5)Exit\nInput: ";
        cin >> user_input;
        switch (user_input) {
            case 1: create_svm();
                break;
            case 2: test_svm();
                break;
            case 3: test_Hog();
                break;
            case 4: detect_objects();
                break;
            case 5: cout << "Goodbye\n";
                break;
        }
    } while (user_input != 5);
}

void create_svm() {
    MachineLearning ml;
    int user_input;
    cout << "What type of kernel do you want to use when training your SVM\n";
    cout << "(1)Linear?\n(2)RBF?\n(3)CHI?\n(4)Poly?\n(5)Sigmoid?\n(6)Continue\n";
    while (user_input != 6) {
        cout << "Kernel type ";
        cin >> user_input;
        if (user_input < 1 || user_input > 6)
            cout << "Invalid input\n";
        else if (user_input != 6)
            svm_kernels.push_back(user_input);
    }
    cout << "What type of feature extraction do you want to use?\n";
    cout << "(1)HOG?\n(2)ORB?\n(3)Continue\n";
    user_input = 0;
    while (user_input != 3) {
        cout << "Feature extraction: ";
        cin >> user_input;
        if (user_input < 1 || user_input > 3)
            cout << "Invalid input\n";
        else if (user_input != 3)
            feature_extraction.push(user_input);
    }
    cout << "Will begin training your SVM\n";
    ml.train_svm();
}

void test_svm() {
    MachineLearning ml;
    string svm_file;
    cout << "What svm file do you wish to test?\n Input: ";
    cin >> svm_file;
    cout << "What type of feature extraction do you want to use?\n";
    cout << "(1)HOG?\n(2)ORB?\n(3)Continue\n";
    int user_input;
    while (user_input != 3 && feature_extraction.size() < 1) {
        cout << "Feature extraction: ";
        cin >> user_input;
        if (user_input < 1 || user_input > 3)
            cout << "Invalid input\n";
        else if (user_input != 3)
            feature_extraction.push(user_input);
    }
    while (!feature_extraction.empty()) {
        ml.Testing(svm_file);
        trainingdata.clear();
        labels.release();
        N = 0;
        feature_extraction.pop();
    }
}

void detect_objects(){
        string modelLibPath;
        cout << "What SVM do you want to use?\nInput: ";
        cin >> modelLibPath;
        Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
        thread thread_1(HogObjectDetection, svm);
        thread thread_2(catergorize);

        thread_1.join();
        thread_2.join();
}

MachineLearning::MachineLearning(void) {//Class constructor

}

MachineLearning::~MachineLearning(void) {
    //Class destructor
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
            cout << "\nEntering: " << newPath << endl;
            TraverseDirectory(newPath);
            //cout << N << "\n";
            label_ = N++;
        } else if (dirEntry->d_type == DT_REG && dirEntry->d_name[0] != '.') {

            Mat greyImgMat = MachineLearning::ProcessImage(path, dirEntry->d_name);
            if (testing == 2) {
                sample_neg(greyImgMat);
            }
            int flag = feature_extraction.front();
            switch (flag) {
                case 1:MachineLearning::HogFeatureExtraction(greyImgMat, label_);
                    break;
                case 2:MachineLearning::OrbFeatureExtraction(greyImgMat);
            }
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
    //convert image to grayscale
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    src.release();
    return gray;

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
    if (testing == 1) {
        img_label.push(N);
    }
    size_++;
    extraction_type = "HOG";

}

void MachineLearning::OrbFeatureExtraction(Mat ImgMat) {
    Ptr<ORB> detector = ORB::create(500, 1.2, 3, 15, 0);
    vector<KeyPoint> keypoints_1;
    //-- Draw keypoints
    cv::Mat img_keypoints_1;
    //detect keypoints
    detector->detect(ImgMat, keypoints_1);
    //describe keypoints
    detector->compute(ImgMat, keypoints_1, img_keypoints_1);
    drawKeypoints(ImgMat, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
    //imshow("test", img_keypoints_1);
    // waitKey();
    if (!img_keypoints_1.empty()) {
        trainingdata.push_back(img_keypoints_1.reshape(1, 1));
        labels.push_back(N);
        size_++;

    }
    if (testing == 1) {
        img_label.push(N);
    }
    extraction_type = "ORB";
}

void MachineLearning::train_svm() {
    MachineLearning ml;
    while (!feature_extraction.empty()) {
        for (int i = 0; i < svm_kernels.size(); i++) {
            trainingdata.clear();
            labels.release();
            string IMAGES_DIR = "/home/" + user + "/Pictures/Training";
            N = 0;
            ml.TraverseDirectory(IMAGES_DIR);
            labels.convertTo(labels, CV_32SC1);
            //trainingdata.convertTo(trainingdata, CV_32FC1);
            Mat train_data;
            Ptr<TrainData> td;
            Ptr<SVM> svm;
            convert_to_ml(trainingdata, train_data);
            trainingdata.clear();
            //Set up SVM's parameters
            svm = SVM::create();
            ml.set_kernal(svm, svm_kernels.at(i));
            td = TrainData::create(train_data, ROW_SAMPLE, labels);
            cout << "Beginning Training Process\n";
            cout << "Training a SVM with a " << svm_type << " kernel using " << extraction_type << "\n";
            svm->trainAuto(td, 10);
            string svm_file = svm_type + extraction_type + ".xml";
            svm->save(svm_file);
            label_ = 0;
            N = 0;
            doorcounter = 0;
            train_data.release();
            labels.release();
            src.release();
            ml.Testing(svm_file);
            testing = 0;
        }
        feature_extraction.pop();
    }

}

void MachineLearning::set_kernal(Ptr<SVM> svm, int flag) {
    switch (flag) {
        case 1: svm->setKernel(SVM::LINEAR);
            svm_type = "Linear";
            break;
        case 2: svm->setKernel(SVM::RBF);
            svm_type = "RBF";
            break;
        case 3: svm->setKernel(SVM::CHI2);
            svm_type = "CHI";
            break;
        case 4: svm->setKernel(SVM::POLY);
            svm->setDegree(2);
            svm_type = "Poly";
            break;
        case 5: svm->setKernel(SVM::SIGMOID);
            svm_type = "SIGMOID";
            break;
    }
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

void MachineLearning::Testing(string svm) {
    testing = 1;
    std::ofstream results;

    const string modelLibPath = svm;
    //Ptr<SVM> Svm = SVM::create();
    Ptr<SVM> Svm = StatModel::load<SVM>(modelLibPath);
    //ml.IMAGES_DIR = "/home/" + user + "/Desktop/Doors";
    string IMAGES_DIR = "/home/" + user + "/Pictures/Test";
    TraverseDirectory(IMAGES_DIR);

    //trainingdata.convertTo(trainingdata, CV_32FC1);
    bool c = Svm->isClassifier();
    bool t = Svm->isTrained();
    cout << c << " " << t << "\n";
    Mat train_data;
    convert_to_ml(trainingdata, train_data);
    Mat res;
    Svm->predict(train_data, res, 4);
    float answer = 0;
    for (int i = 0; i < res.rows; i++) {

        if (res.at<float>(i, 0) == 0)
            doorcounter++;
        //        else
        //            negcounter++;
        if (res.at<float>(i, 0) == (float) img_label.front())
            answer++;
        cout << res.at<float>(i, 0) << " " << proc_img.front() << " " << img_label.front() << "\n"; // << "Prediction: " << p << "\n";
        proc_img.pop();
        img_label.pop();
    }
    float correct = answer / (res.rows + 1);
    cout << "Number of doors: " << doorcounter << "/" << res.rows + 1 << "\n";
    //cout << "Number of other objects: " << negcounter << "/" << res.rows << "\n";
    cout << "Total correct " << correct * 100.0 << "%\n";
    if (iteration == 1)
        results.open("Results.txt", std::ofstream::out);
    else
        results.open("Results.txt", std::ofstream::app);
    results << "This svm accurately predicted " << answer << "/" << res.rows + 1 << " which equates to " << correct * 100.0 << "%\n";
    results << "\n";
    results.close();
    //}
    res.release();
    train_data.release();
}

void HogObjectDetection(Ptr<SVM> svm) {
    running = true;
    MachineLearning ml;
    VideoCapture cap;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    Scalar window(255, 0, 0);
    vector< Rect > doors;
    ml.get_svm_detector(svm, hog_detector);

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

        //draw = img.clone();

        doors.clear();
        hog.detectMultiScale(img, doors, true);
        //people.detectMultiScale(img, locations);
        ml.draw_locations(img, doors, reference, "door");
        //cout << "door\n";
        imshow("video capture", img);
        //trainingdata.release();
        // }
        //else{
        //imshow("video capture", img);
        //}
        //}


        if (waitKey(10) >= 0)
            break;
    }
    destroyAllWindows();
    cap.release();
    running = false;
    while (!categorized_objects.empty()) {
        //namedWindow("Display window", CV_WINDOW_AUTOSIZE);
        cout << "done\n";
        imshow("door", categorized_objects.front());
        waitKey(0);
        categorized_labels.pop();
        categorized_objects.pop();
    }
}

void catergorize() {
    const string modelLibPath = "LinearHOG.xml";
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    MachineLearning ml;
    while (running) {
        if (!detected_objects.empty()) {
            Mat object, train_data, res;
            resize(detected_objects.front(), object, Size(64, 128));
            detected_objects.pop();
            ml.HogFeatureExtraction(object, -1);
            ml.convert_to_ml(trainingdata, train_data);
            trainingdata.clear();

            //Roi.convertTo(Roi, CV_32FC1);
            svm->predict(train_data, res, 4);
            //cout << res.at<float>(i,0) << " " << "locations" << locations.size() << "\n";
            for (int j = 0; j < res.rows; j++) {
                cout << res.at<float>(j, 0) << " " << "res " << res.rows << "\n";
                //Show ROI
                if (res.at<float>(j, 0) == 1) {
                    categorized_labels.push(1);
                    categorized_objects.push(object);
                }
            }
        }
    }
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
        }
        detected_objects.push(img.clone());

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
    const string modelLibPath = "LinearHOG.xml";
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    ml.get_svm_detector(svm, hog_detector);
    //        if (!cap.isOpened())
    //            return -1;
    Mat draw, img;
    HOGDescriptor hog; //(Size( 90, 160 ), Size(16, 16), Size(8, 8), Size(8, 8), 9);
    hog.winSize = Size(64, 128);
    hog.gammaCorrection = true;
    hog.setSVMDetector(hog_detector);
    src = greyimg;
    resize(src, src, Size(256, 512));
    resize(display, display, Size(256, 512));
    namedWindow("Step 2 draw Rectangle", WINDOW_AUTOSIZE);
    imshow("Step 2 draw Rectangle", display);
    waitKey(100);
    imwrite("Step2.JPG", display);
    locations.clear();
    hog.detectMultiScale(src, locations, false);
    for (int i = 0; i < locations.size(); i++) {
        Rect windows(locations.at(i).x, locations.at(i).y, locations.at(i).width, locations.at(i).height);
        img = src(windows).clone();
        Mat Roi = src(windows);
        Mat extract;
        resize(Roi, extract, Size(64, 128));
        Mat res;
        Mat train_data;
        ml.HogFeatureExtraction(extract, -1);
        ml.convert_to_ml(trainingdata, train_data);
        trainingdata.clear();
        //Roi.convertTo(Roi, CV_32FC1);
        svm->predict(train_data, res, 4);
        //cout << res.at<float>(i,0) << " " << "locations" << locations.size() << "\n";
        for (int j = 0; j < res.rows; j++) {
            cout << res.at<float>(j, 0) << " " << "res " << res.rows << " count " << neg_count << "\n";
            //Show ROI
            if (res.at<float>(j, 0) == 1) {
                namedWindow("Step 4 Draw selected Roi", WINDOW_AUTOSIZE);
                imshow("Step 4 Draw selected Roi", Roi);
                waitKey(100);
                imwrite("Step4.JPG", Roi);
                int neg = neg_count;
                stringstream ss;
                ss << neg;
                string str = ss.str();
                string destination = "/home/thomas/Pictures/Training/other/" + str + ".JPG";
                imwrite(destination, Roi);
                neg_count++;
            }
        }
        res.release();
        train_data.release();
        extract.release();
    }
    Mat DrawResultHere = display.clone();
    // Show  rectangle
    namedWindow("Step 2 draw Rectangle", WINDOW_AUTOSIZE);
    imshow("Step 2 draw Rectangle", DrawResultHere);
    waitKey(100);
    imwrite("Step2.JPG", DrawResultHere);
}

void test_Hog() {
    MachineLearning ml;
    string IMAGES_DIR = "/home/" + user + "/Pictures/Training/other";
    testing = 2;
    ml.TraverseDirectory(IMAGES_DIR);
    ml.train_svm();
    cout << neg_count;
}
