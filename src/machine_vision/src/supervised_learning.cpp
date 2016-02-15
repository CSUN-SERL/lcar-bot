#include "machine_learning.h"

//#include <sstream>

void SetLabel(cv::Mat& im, cv::Rect r, const std::string label);
void NegativeMiningSetup();
void NegativeMining(Mat greyimg);
void CategorizeObjects();
void HogObjectDetection(Ptr<SVM> svm);
void Run();
void CreateSVM();
void ClassificationTestingSetup();
void DetectObjects();
void StaticHogTestSetup();
void StaticTest(Mat img);
void ResetGlobals();

//Globals for object categorization
//Tracks when HogObjectdetection is running
bool running = false;
//A list of all objects detect during HogObjectDetection
queue<Mat> detected_objects;
//Not yet implemented, supposed to be used for returning categorized objects
queue<Mat> categorized_objects;
//Not yet implemented, supposed to be used for returning labels of categorized
//objects. Meant to be used with categorized_objects.
queue<int> categorized_labels;

//Globals for user control
//A variable tracking which kernel the user wants to be used in SVM training.
int svm_kernel;
//A variable tracking which type of feature extraction the user wants to use with
//SVM training.
int feature_extraction;
//File name of the SVM
string svm_name;

//Globals mats used for training
//SVM object that wold holds weights
Ptr<SVM> svm;
//List of labels to be used for SVM training.
Mat labels;
//List of features extracted from training images, meant to be used with
//labels Mat.
vector<Mat> trainingdata;
//Use to declare image size and Hog winSize.
Size img_size = Size(64, 128);

//Copys of current image
//Current image being processed
Mat src;
//Another copy of the image being processed
Mat display;

//Global queues used to track img names and labels for testing
//A queue of image names
queue<string> proc_img;
//A queue of the correct label of an image, to be used with proc_image
queue<int> img_label;

//Parameter used to determine which form of testing is being done.
//(1) Classification testing
//(2) Negative mining
//(3) Static testing
int testing;

//N is label tracker, size_ is for total number of images read
int N, size_;

//Variables to be used in the Classification testing process
float door_counter, neg_counter, window_counter;
//Variables to be used in Static testing process.
float static_correct, static_total;

//what the fuck are these
int number = 0;
int label_ = 0;
int neg_count = 2315;
int object = -1;

//Strings that are resued
//Retrieve the desktop user.
string user = getenv("USER");
//Stores the kernel type of the svm.
string svm_type;
//Stores the extraction type of the svm.
string extraction_type;

//May later be used for testing purposes
float best_correct;
string best_svm;


int iteration; //number corresponds to svm
int iteration_2; //number correspons to extraction type

int main(int argc, char * argv[]) {
    Run();
    ResetGlobals();
    return 0;
}

MachineLearning::MachineLearning(void) {//Class constructor

}

MachineLearning::~MachineLearning(void) {//Class destructor

}

//Menu that allows for quick testing
void Run() {
    int user_input = 0;
    do {
        cout << "What are you planning on doing?\n";
        cout << "(1)Create a SVM\n(2)Run negative mining\n(3)Run classification testing\n(4)Run static testing\n(5)Run object detection\n(6)Exit\nInput: ";
        cin >> user_input;
        switch (user_input) {
            case 1: CreateSVM();
                break;
            case 2: NegativeMiningSetup();
                break;
            case 3: ClassificationTestingSetup();
                break;
            case 4: StaticHogTestSetup();cout << static_correct << "/" << static_total << " = " << (static_correct/static_total) * 100 << "%\n";//DetectObjects();
                break;
            case 5: StaticHogTestSetup();cout << static_correct << "/" << static_total << " = " << (static_correct/static_total) * 100 << "%\n";
                break;
            case 6: cout << "Goodbye\n";
                break;
        }
    } while (user_input != 6);
}

//Retrieve the parameters used to train an svm, optimize to allow multiple svms to be made at once
void CreateSVM() {
    MachineLearning ml;
    int user_input;
    cout << "What do you want to call the SVM?";
    cin >> svm_name;
    cout << "What type of kernel do you want to use when training your SVM\n";
    cout << "(1)Linear?\n(2)RBF?\n(3)CHI?\n(4)Poly?\n(5)Sigmoid?\n";
    cout << "Kernel type ";
    cin >> user_input;
    if (user_input < 1 || user_input > 5){
        cout << "Invalid input\n";
        CreateSVM();
    }
   svm_kernel = user_input;

    cout << "What type of feature extraction do you want to use?\n";
    cout << "(1)HOG?\n(2)ORB?\n";
    user_input = 0;
    cout << "Feature extraction: ";
    cin >> user_input;
    if (user_input < 1 || user_input > 2)
        cout << "Invalid input\n";

    feature_extraction = user_input;

    cout << "Will begin training your SVM\n";
    ml.TrainSvm();
}

//Create and save a svm based on the paremeters entered by the user
void MachineLearning::TrainSvm() {
    MachineLearning ml;
    ResetGlobals();
    string IMAGES_DIR = "/home/" + user + "/Pictures/Training";
    ml.TraverseDirectory(IMAGES_DIR);
    labels.convertTo(labels, CV_32SC1);
    Mat train_data;
    svm = SVM::create();;
    ConvertToMl(trainingdata, train_data);
    trainingdata.clear();
    //Set up SVM's parameters
    ml.SetKernal(svm, svm_kernel);
    //Set up training data
    Ptr<TrainData> td = TrainData::create(train_data, ROW_SAMPLE, labels);
    cout << "Beginning Training Process\n";
    cout << "Training a SVM with a " << svm_type << " kernel using " << extraction_type << "\n";
    svm->trainAuto(td, 10);
    string svm_file = svm_name + ".xml";
    svm->save(svm_file);
    ResetGlobals();
}

//Method that sets parameters for negative mining
void NegativeMiningSetup() {
    MachineLearning ml;
    string IMAGES_DIR = "/home/" + user + "/Pictures/Training/other";
    testing = 2;
    ml.TraverseDirectory(IMAGES_DIR);
    ml.TrainSvm();
    cout << neg_count;
}

//Perform negtative mining to reduce false positives
void NegativeMining(Mat greyimg) {
    MachineLearning ml;
    vector< float > hog_detector;
    vector< Rect > locations;
    svm_name = "";
    cout << "Run negative mining on which SVM?";
    cin >> svm_name;
    const string modelLibPath = svm_name;
    svm = StatModel::load<SVM>(modelLibPath);
    ml.GetSvmDetector(svm, hog_detector);
    Mat img;
    HOGDescriptor hog;
    hog.winSize = img_size;
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
    hog.detectMultiScale(src, locations, true);
    for (int i = 0; i < locations.size(); i++) {
        Rect windows(locations.at(i).x, locations.at(i).y, locations.at(i).width, locations.at(i).height);
        img = src(windows).clone();
        Mat Roi = src(windows);
        Mat extract;
        resize(Roi, extract, img_size);
        Mat res;
        Mat train_data;
        ml.HogFeatureExtraction(extract, -1);
        ml.ConvertToMl(trainingdata, train_data);
        trainingdata.clear();
        svm->predict(train_data, res, 4);
        for (int j = 0; j < res.rows; j++) {
            cout << res.at<float>(j, 0) << " " << "res " << res.rows << " count " << neg_count << "\n";
            //Show ROI
            if (res.at<float>(j, 0) == 0) {
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
        ResetGlobals();
    }
    Mat DrawResultHere = display.clone();
    // Show  rectangle
    namedWindow("Step 2 draw Rectangle", WINDOW_AUTOSIZE);
    imshow("Step 2 draw Rectangle", DrawResultHere);
    waitKey(100);
    imwrite("Step2.JPG", DrawResultHere);
}

//Prep svm for classification testing
void ClassificationTestingSetup() {
    ResetGlobals();
    MachineLearning ml;
    svm_name = "";
    cout << "What svm file do you wish to test?\n Input: ";
    cin >> svm_name;
    cout << "What type of feature extraction do you want to use?\n";
    cout << "(1)HOG?\n(2)ORB?\n";
    int user_input;
    cout << "Feature extraction: ";
    cin >> user_input;
    if (user_input < 1 || user_input > 3)
        cout << "Invalid input\n";
    else if (user_input > 0 && user_input < 3)
        feature_extraction = user_input;

    ml.ClassificationTesting(svm_name);
    ResetGlobals();
}

//The goal of this method is detect and classify objects in real time
void DetectObjects() {
    string modelLibPath;
    cout << "What SVM do you want to use?\nInput: ";
    cin >> modelLibPath;
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    thread thread_1(HogObjectDetection, svm);
    thread thread_2(CategorizeObjects);

    thread_1.join();
    thread_2.join();
}

//Convert a vector mat into a single mat
void MachineLearning::ConvertToMl(const std::vector< cv::Mat > & train_samples, cv::Mat& trainData) {
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
void MachineLearning::GetSvmDetector(const Ptr<SVM>& svm, vector< float > & hog_detector) {
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

//Traverse through a directory to find all images then perform image processing and feature extraction
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
            cout << N << "\n";
            N++;
        } else if (dirEntry->d_type == DT_REG && dirEntry->d_name[0] != '.') {

            Mat greyImgMat = MachineLearning::ProcessImage(path, dirEntry->d_name);
            if (testing == 2) {
                NegativeMining(greyImgMat);
            }
            if (testing == 3) {
                StaticTest(greyImgMat);
            }
            int flag = feature_extraction;
            switch (flag) {
                case 1:
                    MachineLearning::HogFeatureExtraction(greyImgMat, N);
                    break;
                case 2:MachineLearning::OrbFeatureExtraction(greyImgMat);
            }
        }
    }

    closedir(tree);
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
        labels.push_back(N); //labels.push_back(label);
    //cout << label << "\n";
    if (testing == 1) {
        img_label.push(N);
    }
    size_++;
    extraction_type = "HOG";

}

//Feature extraction based on ORB
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



//Set kernel based on previous user input
void MachineLearning::SetKernal(Ptr<SVM> svm, int flag) {
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

//Attatch a  label to a detected object
void SetLabel(cv::Mat& im, cv::Rect r, const std::string label) {
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

//Classification testing, see how accurate the svm is at classifying objects
void MachineLearning::ClassificationTesting(string svm_path) {
    testing = 1;
    //std::ofstream results;
    const string modelLibPath = svm_path;
    svm = StatModel::load<SVM>(modelLibPath);
    string IMAGES_DIR = "/home/" + user + "/Pictures/Classification_Test";
    TraverseDirectory(IMAGES_DIR);
    //trainingdata.convertTo(trainingdata, CV_32FC1);
    //bool c = Svm->isClassifier();
    //bool t = Svm->isTrained();
    //cout << c << " " << t << "\n";
    Mat train_data;
    ConvertToMl(trainingdata, train_data);
    Mat res;
    svm->predict(train_data, res, 4);
    float answer = 0;
    for (int i = 0; i < res.rows; i++) {
        if (res.at<float>(i, 0) == 0)
            door_counter++;
        if (res.at<float>(i, 0) == (float) img_label.front())
            answer++;
        cout << res.at<float>(i, 0) << " " << proc_img.front() << " " << img_label.front() << "\n"; // << "Prediction: " << p << "\n";
        proc_img.pop();
        img_label.pop();
    }
    float correct = answer / (res.rows + 1);
    cout << "Total correct " << correct * 100.0 << "%\n";
//    if (iteration == 1)
//        results.open("Results.txt", std::ofstream::out);
//    else
//        results.open("Results.txt", std::ofstream::app);
//    results << "This svm accurately predicted " << answer << "/" << res.rows + 1 << " which equates to " << correct * 100.0 << "%\n";
//    results << "\n";
//    results.close();
//    //}
    ResetGlobals();
}

//Perform real time HOG object detection
void HogObjectDetection(Ptr<SVM> current_svm) {
    running = true;
    MachineLearning ml;
    VideoCapture cap;
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    Scalar window(255, 0, 0);
    vector< Rect > objects;
    ml.GetSvmDetector(current_svm, hog_detector);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
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
        ml.DrawLocations(img, objects, reference, "door");
        imshow("video capture", img);
        if (waitKey(10) >= 0)
            break;
    }
}

//While objects are being detected catergorize them and store the ones that are desired object
void CategorizeObjects() {
    const string modelLibPath = svm_name;
    svm = StatModel::load<SVM>(modelLibPath);
    MachineLearning ml;
    while (true) {
        if (!detected_objects.empty()) {
            Mat object, train_data, res;
            resize(detected_objects.front(), object, img_size);
            detected_objects.pop();
            ml.HogFeatureExtraction(object, -1);
            ml.ConvertToMl(trainingdata, train_data);
            trainingdata.clear();

            //Roi.convertTo(Roi, CV_32FC1);
            svm->predict(train_data, res, 4);
            //cout << res.at<float>(i,0) << " " << "locations" << locations.size() << "\n";
            for (int j = 0; j < res.rows; j++) {
                cout << res.at<float>(j, 0) << " " << "res " << res.rows << "\n";
                //Show ROI
                //                if (res.at<float>(j, 0) == 1) {
                //                    categorized_labels.push(1);
                //                    categorized_objects.push(object);
                //                }
            }
        }
    }
}

//Draw detected objects on a window
void MachineLearning::DrawLocations(Mat & img, const vector< Rect > & found, const Scalar & color, string label) {
    if (!found.empty()) {
//        vector<Rect> found_filtered;
//        size_t i, j;
//        for (i = 0; i < found.size(); i++) {
//            Rect r = found[i];
//            for (j = 0; j < found.size(); j++)
//                if (j != i && (r & found[j]) == r)
//                    break;
//            if (j == found.size())
//                found_filtered.push_back(r);
//        }
//        for (i = 0; i < found_filtered.size(); i++) {
//            Rect r = found_filtered[i];
//            r.x += cvRound(r.width * 0.1);
//            r.width = cvRound(r.width * 0.8);
//            r.y += cvRound(r.height * 0.06);
//            r.height = cvRound(r.height * 0.9);
//            rectangle(img, r.tl(), r.br(), cv::Scalar(0, 255, 0), 2);
//        }


                vector< Rect >::const_iterator loc = found.begin();
                vector< Rect >::const_iterator end = found.end();
                for (; loc != end; ++loc) {
                    rectangle(img, *loc, color, 2);
                    SetLabel(img, *loc, label);
                    detected_objects.push(img.clone());

                }


    }
}

//Sets parameters for static testing
void StaticHogTestSetup() {
    ResetGlobals();
    cout << "Enter SVM to test: ";
    cin >> svm_name;
    MachineLearning ml;
    string IMAGES_DIR = "/home/" + user + "/Pictures/Static_Test";
    testing = 3;
    ml.TraverseDirectory(IMAGES_DIR);
}

//Attempt to detect doors on static images
void StaticTest(Mat img) {
    MachineLearning ml;
    const string modelLibPath = svm_name;
    Ptr<SVM> svm = StatModel::load<SVM>(modelLibPath);
    vector< float > hog_detector;
    Scalar reference(0, 255, 0);
    vector< Rect > objects;
    vector< Rect > door_objects;
    ml.GetSvmDetector(svm, hog_detector);
    HOGDescriptor hog;
    hog.winSize = img_size;
    hog.setSVMDetector(hog_detector);
    namedWindow("video capture", CV_WINDOW_AUTOSIZE);
    src = img;
    objects.clear();
    door_objects.clear();
    resize(img, img, Size(256, 512));
    resize(display, display, Size(256, 512));
    hog.gammaCorrection = true;
    hog.detectMultiScale(img, objects, true);
    //hog.detectMultiScale(display, objects, 0, Size(4,4), Size(0,0), 1.03, 2.0, true);
    for (int i = 0; i < objects.size(); i++) {
        Rect windows(objects.at(i).x, objects.at(i).y, objects.at(i).width, objects.at(i).height);
        Mat Roi = display(windows);
        Mat extract;
        resize(Roi, extract, img_size);
        Mat res;
        Mat train_data;
        ml.HogFeatureExtraction(extract, -1);
        ml.ConvertToMl(trainingdata, train_data);
        trainingdata.clear();
        svm->predict(train_data, res, 4);
        for (int j = 0; j < res.rows; j++) {
            cout << res.at<float>(j, 0) << " " << "res " << res.rows << " count " << neg_count << "\n";
            //Show ROI
            if (res.at<float>(j, 0) == 0) {
                door_objects.push_back(objects.at(i));
            }
        }
        res.release();
        train_data.release();
        extract.release();
    }
    ml.DrawLocations(display, door_objects, reference, "door");
    imshow("detected doors", display);
    char k = waitKey(0);
    if(k == 'y')
        static_correct++;
    static_total++;
}

void ResetGlobals(){
    labels.release();
    src.release();
    trainingdata.clear();
    N = 0;
    label_ = 0;
    svm_name = "";
    svm.release();
}
