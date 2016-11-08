
#ifndef IMAGE_H
#define IMAGE_H

#include <QDir>
#include <QStringBuilder>
#include <QImage>
#include <QPixmap>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

namespace rqt_gcs
{

namespace img 
{   
    QImage matToQimg(const cv::Mat& in);
    QImage rosImgToQimg(const sensor_msgs::Image& in);
    QImage rosImgToQimg(const sensor_msgs::ImageConstPtr& in);
    QPixmap matToQpixmap(const cv::Mat& in);
    QPixmap rosImgToQpixmap(const sensor_msgs::Image& in);
    QPixmap rosImgToQpixmap(const sensor_msgs::ImageConstPtr& in);
    cv::Mat qImgToMat(const QImage& in);
    cv::Mat rosImgToMat(const sensor_msgs::Image& in);
    void qImgToRosImg(const QImage& in, sensor_msgs::Image& out);
    sensor_msgs::ImagePtr matToRosImg(const cv::Mat& in);

    bool saveImage(QString& path, QString& file, const QImage& image);
    bool saveImage(QString& path, QString& file, const cv::Mat& image);
    bool saveImage(QString& path, QString& file, const sensor_msgs::Image& image);
    
    int numImagesInDir(QString& dir_path);
    int imgNumFromFile(QString& file);
    QString getImgBasePath(QString& file_path);
    
}

}

#endif /* IMAGE_H */

