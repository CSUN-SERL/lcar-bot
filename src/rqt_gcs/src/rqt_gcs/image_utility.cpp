
#include "util/image.h"

namespace rqt_gcs
{
    
namespace image_util
{

    QImage matToQimg(const cv::Mat& in, QImage::Format format)
    {
        return QImage(in.data, in.cols, in.rows, in.step, format);
    }

    QImage rosImgToQimg(const sensor_msgs::Image& in, QImage::Format format)
    {
        return QImage(in.data.data(), in.width, in.height, in.step, format).rgbSwapped();
    }

    QImage rosImgToQimg(const sensor_msgs::ImageConstPtr& in, QImage::Format format)
    {   
        return QImage(in->data.data(), in->width, in->height, in->step, format).rgbSwapped();
    }

    cv::Mat qImgToMat(const QImage& in, int format)
    {   
        QImage img = in.rgbSwapped();
        return cv::Mat(img.height(), img.width(), format, (uchar *)img.bits(), img.bytesPerLine());
    }

    cv::Mat rosImgToMat(const sensor_msgs::Image& in, int format)
    {
        return cv::Mat(in.height, in.width, format, (uchar *)in.data.data(), in.step);
    }

    sensor_msgs::ImagePtr qImgToRosImg(const QImage& in, std::string format)
    {   
        QImage temp = in.rgbSwapped();
        //todo handle different image encodings;
        cv::Mat img(temp.height(), temp.width(), CV_8UC3, (uchar *)temp.bits(), temp.bytesPerLine());
        return matToRosImg(img, format);
    }

    sensor_msgs::ImagePtr matToRosImg(const cv::Mat& in, std::string format)
    {
        return cv_bridge::CvImage(std_msgs::Header(), format, in).toImageMsg();
    }

    bool saveImage(QString& path, QString& file, const QImage& image)
    {
        QDir dir(path);
        if(!dir.exists())
            dir.mkpath(path);

        QString full_path = dir.canonicalPath().append("/" % file);
        return image.save(full_path, "jpg", -1);
    }

    bool saveImage(QString& path, QString& file, const cv::Mat& image)
    {
        return saveImage(path, file, matToQimg(image));
    }

    bool saveImage(QString& path, QString& file, const sensor_msgs::Image& image)
    {
        return saveImage(path, file, rosImgToQimg(image));
    }

    int numImagesInDir(QString& dir_path)
    {
        QDir dir(dir_path);
        dir.setNameFilters(QStringList() << "*.jpg");
        return dir.entryList().size();
    }

    int imgNumFromFile(QString& file)
    {
        int start = file.indexOf("img_");
        QString temp = file.mid(start+4,4);
        int stop = temp.indexOf(".");
        return temp.mid(0, stop).toInt();
    }

    QString getImgBasePath(QString& file_path)
    {
        int start = file_path.indexOf("/img_");
        return file_path.mid(0,start);
    }

} //namespace image_util
    
} //namespace rqt_gcs
