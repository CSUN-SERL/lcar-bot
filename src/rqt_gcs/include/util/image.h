
#ifndef IMAGE_H
#define IMAGE_H

#include <QDir>
#include <QStringBuilder>
#include <QImage>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

namespace rqt_gcs
{

namespace img 
{   
    
    extern QString image_root_dir_;
    
    QImage matToQimg(const cv::Mat& in, QImage::Format format = QImage::Format_RGB888);
    QImage rosImgToQimg(const sensor_msgs::Image& in, QImage::Format format = QImage::Format_RGB888);
    QImage rosImgToQimg(const sensor_msgs::ImageConstPtr& in, QImage::Format format = QImage::Format_RGB888);
    cv::Mat qImgToMat(const QImage& in, int fomat = CV_8UC3);
    cv::Mat rosImgToMat(const sensor_msgs::Image& in, int format = CV_8UC3);
    sensor_msgs::ImagePtr qImgToRosImg(const QImage& in, std::string format = "bgr8");
    sensor_msgs::ImagePtr matToRosImg(const cv::Mat& in, std::string format = "bgr8");

    bool saveImage(QString& path, QString& file, const QImage& image);
    bool saveImage(QString& path, QString& file, const cv::Mat& image);
    bool saveImage(QString& path, QString& file, const sensor_msgs::Image& image);
    
    int numImagesInDir(QString& dir_path);
    int imgNumFromFile(QString& file);
    QString getImgBasePath(QString& file_path);
    
}

}

#endif /* IMAGE_H */

