#include "machine_vision.h"

Mat labels;
Mat trainingdata;
Mat src;
queue<string> proc_img;
int testing;
int N, size;
float doorcounter, negcounter;
int number = 0;


int main(int argc, char * argv[]) {
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
            MachineLearning::ExtractFeatures(greyImgMat);
        }
    }

    closedir(tree);
}

Mat MachineLearning::ProcessImage(string path, string file) {


    if (path != " " && file != " ") {
        cout << "grey scaling: " << file << endl;
        src = imread(path + "/" + file);
        if (testing == 1) {
            proc_img.push(file);
        }
    }
    if (!src.data) {
        cout << "no image" << endl;
        exit(0);
    }

    resize(src, src, Size(96, 160));

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

void MachineLearning::ExtractFeatures(Mat ImgMat) {

    //apply canny edge detector
    Mat bw;
    cv::Canny(ImgMat, bw, 0, 50, 5);

    //Find Houghlines
    std::vector<cv::Vec4i> lines_vec;
    HoughLinesP(bw, lines_vec, 1, CV_PI / 180, 100, 30, 10);

    for (size_t i = 0; i < lines_vec.size(); i++) {
        line(src, Point(lines_vec[i][0], lines_vec[i][1]), Point(lines_vec[i][2], lines_vec[i][3]), Scalar(0, 0, 255), 3, 8);
    }

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat drawing = Mat::zeros(bw.size(), CV_8UC3);
    Scalar colors[3];
    colors[0] = Scalar(255, 0, 0);
    colors[1] = Scalar(0, 255, 0);
    colors[2] = Scalar(0, 0, 255);
    for (int i = 0; i < contours.size(); i++) {
        drawContours(src, contours, i, colors[i % 3]);
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
    drawKeypoints(src, keypoints_1, img_keypoints_1, Scalar::all(-1), DrawMatchesFlags::DEFAULT);

    if (!img_keypoints_1.empty()) {
        trainingdata.push_back(img_keypoints_1.reshape(1, 1));
        labels.push_back(N);
        size++;
    }
}

void MachineLearning::Testing(string test_dir) {
    testing = 1;
    const string modelLibPath = "SVM.yaml";
    Ptr<SVM> Svm = StatModel::load<SVM>(modelLibPath);
    TraverseDirectory(test_dir);

    trainingdata.convertTo(trainingdata, CV_32FC1);
    bool c = Svm->isClassifier();
    bool t = Svm->isTrained();
    cout << c << " " << t << "\n";
    Mat res;
    float p = Svm->predict(trainingdata, res, 4);


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
