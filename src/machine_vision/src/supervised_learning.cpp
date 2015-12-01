#include <opencv2/objdetect/objdetect.hpp>

#include "machine_learning.h"

Mat labels;
Mat trainingdata;
Mat src;
queue<string> proc_img;
int testing;
int N, size;
float counter;
int number = 0;

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector);
void OrbDetection(Ptr<SVM> svm);

int main(int argc, char * argv[]) {
    MachineLearning ml;
//        ml.IMAGES_DIR = "/home/thomas/NetBeansProjects/Training";
//        ml.TraverseDirectory(ml.IMAGES_DIR);
//        labels.convertTo(labels, CV_32SC1);
//        trainingdata.convertTo(trainingdata, CV_32FC1);
//        // Set up SVM's parameters
//        Ptr<SVM> svm = SVM::create();
//        svm->setKernel(SVM::LINEAR);
//        //svm->setTermCriteria(TermCriteria(CV_TERMCRIT_ITER, 100, 1e-6));
//        Ptr<TrainData> td = TrainData::create(trainingdata, ROW_SAMPLE, labels);
//        cout << "inside main training\n";
//        svm->trainAuto(td, 10);
//        svm->save("SVM.yaml");
//        cout << labels;
//        trainingdata.release();
//        //
//    //    //    /*****************************TESTING*************************************/
    ml.Testing(ml);
//    const string modelLibPath = "SVM.yaml";
//    //Ptr<SVM> Svm = SVM::create();
  //  Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    //ml.Hog(svm);
    //OrbDetection(svm);

    trainingdata.release();
    labels.release();
    src.release();

    return 0;
}

MachineLearning::MachineLearning(void) {//Class constructor
    IMAGES_DIR = "";

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
            TraverseDirectory(newPath);
            cout << N << "\n";
            N++;
        } else if (dirEntry->d_type == DT_REG && dirEntry->d_name[0] != '.') {
            Mat greyImgMat = MachineLearning::ProcessImage(path, dirEntry->d_name);
            //MachineLearning::ExtractFeatures(greyImgMat, dirEntry->d_name);
            MachineLearning::HogFeatureExtraction(greyImgMat);
        }
    }

    closedir(tree);
}

Mat MachineLearning::ProcessImage(string path, string file) {

    cout << "grey scaling: " << file << endl;
    src = imread(path + "/" + file);

    if (testing == 1) {
        proc_img.push(file);
    }

    if (!src.data) {
        cout << "no image" << endl;
        exit(0);
    }

    resize(src, src, Size(96, 160));

    //imshow(file, src);
    //waitKey(0);

    //blur image to reduce number of features
    GaussianBlur(src, src, Size(9, 9), 2, 2);
    //erode image
    Mat kernel = Mat::ones(3, 3, CV_8U);
    Mat eroded;
    erode(src, eroded, kernel);
    src = src - eroded;
    //convert image to grayscale
    Mat gray;
    cvtColor(src, gray, CV_BGR2GRAY);
    //Apply Otsu thresholding
    threshold(gray, gray, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);

    dilate(gray, gray, Mat(), Point(-1, -1), 2, 1, 1);


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
    waitKey();

    if (!img_keypoints_1.empty()) {
        trainingdata.push_back(img_keypoints_1.reshape(1, 1));
        labels.push_back(N);
        size++;
    }
}

void MachineLearning::Testing(MachineLearning ml) {
    testing = 1;
    const string modelLibPath = "SVM.yaml";
    //Ptr<SVM> Svm = SVM::create();
    Ptr<SVM> Svm = StatModel::load<SVM>(modelLibPath);
    ml.IMAGES_DIR = "/home/thomas/Desktop/Doors";
    TraverseDirectory(ml.IMAGES_DIR);

    trainingdata.convertTo(trainingdata, CV_32FC1);
    bool c = Svm->isClassifier();
    bool t = Svm->isTrained();
    cout << c << " " << t << "\n";
    Mat res;
    float p = Svm->predict(trainingdata, res, 4);


    for (int i = 0; i < res.rows; i++) {
        if(res.at<float>(i,0) == 0)
            counter++;
        cout << res.row(i) << " " << proc_img.front() << "\n";// << "Prediction: " << p << "\n";
        proc_img.pop();
    }
    float correct = counter/res.rows;
    cout << counter << "/" << res.rows << "\n";
    cout << "Accuracy: " << correct * 100.0 << "%\n";
}

void MachineLearning::Hog(Ptr<SVM> Svm) {
    VideoCapture cap(-1);
    vector< float > hog_detector;
    get_svm_detector(Svm, hog_detector);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1000);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1000);
    //    if (!cap.isOpened())
    //        return -1;
    Mat img;
    HOGDescriptor hog; //(Size(32, 16), Size(8, 8), Size(4, 4), Size(4, 4), 9);

    hog.setSVMDetector(hog_detector);

    //hog.setSVMDetector( HOGDescriptor::getDefaultPeopleDetector() );


    namedWindow("video capture", CV_WINDOW_AUTOSIZE);
    while (true) {
        cap >> img;
        if (!img.data)
            continue;

        vector<Rect> found, found_filtered;
        hog.detectMultiScale(img, found);

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
        imshow("video capture", img);
        if (waitKey(20) >= 0)
            break;
    }
}

void MachineLearning::HogFeatureExtraction(Mat ImgMat) {

    //char SaveHogDesFileName[100] = "Positive.xml";
    vector< vector < float> > v_descriptorsValues;
    vector< vector < Point> > v_locations;

    HOGDescriptor d; //(Size(32, 16), Size(8, 8), Size(4, 4), Size(4, 4), 9);
    vector< float> descriptorsValues;
    vector< Point> locations;
    d.compute(ImgMat, descriptorsValues, Size(8, 8), Size(0, 0), locations);
    v_descriptorsValues.push_back(descriptorsValues);
    v_locations.push_back(locations);
    //imshow("origin", ImgMat);
    Mat Hogfeat;
    Hogfeat.create(descriptorsValues.size(), 1, CV_32FC1);

    for (int i = 0; i < descriptorsValues.size(); i++) {
        Hogfeat.at<float>(i, 0) = descriptorsValues.at(i);

    }

    if (!Hogfeat.empty()) {
        trainingdata.push_back(Hogfeat.reshape(1, 1));
        labels.push_back(N);
        size++;
    }


    //waitKey(0);

    //    FileStorage hogXml(SaveHogDesFileName, FileStorage::WRITE); //FileStorage::READ
    //    //2d vector to Mat
    //    int row = v_descriptorsValues.size();
    //    int col = v_descriptorsValues[0].size();
    //    printf("col=%d, row=%d\n", row, col);
    //    Mat M(row, col, CV_32F);
    //    //save Mat to XML
    //    for (int i = 0; i < row; ++i)
    //        memcpy(&(M.data[col * i * sizeof (float) ]), v_descriptorsValues[i].data(), col * sizeof (float));
    //    //write xml
    //    write(hogXml, "Descriptor_of_images", M);
    //
    //    //write(hogXml, "Descriptor", v_descriptorsValues );
    //    //write(hogXml, "locations", v_locations );
    //    hogXml.release();
}

void get_svm_detector(const Ptr<SVM>& svm, vector< float > & hog_detector) {
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
    CV_Assert(alpha.total() == 1 && svidx.total() == 1 && sv_total == 1);
    CV_Assert((alpha.type() == CV_64F && alpha.at<double>(0) == 1.) ||
            (alpha.type() == CV_32F && alpha.at<float>(0) == 1.f));
    CV_Assert(sv.type() == CV_32F);
    hog_detector.clear();

    hog_detector.resize(sv.cols + 1);
    memcpy(&hog_detector[0], sv.ptr(), sv.cols * sizeof (hog_detector[0]));
    hog_detector[sv.cols] = (float) -rho;
}

void OrbDetection(Ptr<SVM> svm) {
    VideoCapture cap(-1);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1000);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1000);
    //    if (!cap.isOpened())
    //        return -1;
    Mat img;
    vector<KeyPoint> keypoints_1;
    Ptr<ORB> detector = ORB::create(500, 1.2, 3, 15, 0);
    while (true) {

        cap >> img;
        if (!img.data)
            continue;
        cvtColor(img, img, CV_BGR2GRAY);

        vector<Rect> found, found_filtered;
        cv::Mat img_keypoints_1, res;
        //MachineLearning::ExtractFeatures(img, "");

        //detect keypoints
        detector->detect(img, keypoints_1);
        //describe keypoints
        detector->compute(img, keypoints_1, img_keypoints_1);
        drawKeypoints(img, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
        imshow("video capture", img_keypoints_1);
//        resize(img_keypoints_1, img_keypoints_1, Size(640, 480));
//
//        img_keypoints_1.convertTo(img_keypoints_1, CV_32FC1);
//        img_keypoints_1.reshape(1, 1);
//        float p = svm->predict(img_keypoints_1, res, 4);
//
//
//        for (int i = 0; i < res.rows; i++) {
//            cout << res.row(i) << " " << "Prediction: " << p << "\n";
//        }
        if (waitKey(20) >= 0)
            break;

    }
}
