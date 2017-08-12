
#include <gcs/util/image_conversions.h>
#include <sensor_msgs/fill_image.h>

namespace gcs
{
    
namespace image_conversions
{

    QImage matToQimg(const cv::Mat& in)
    {
        QImage::Format f = (in.channels() == 1) ? QImage::Format_Grayscale8
                                                : QImage::Format_RGB888;
        return QImage(in.data, in.cols, in.rows, in.step, f);
    }
       
    QPixmap matToQpixmap(const cv::Mat& in)
    {
        return QPixmap::fromImage(matToQimg(in));
    }

    QImage rosImgToQimg(const sensor_msgs::Image& in)
    {
        bool swap;
        QImage::Format f = (QImage::Format)rosEncToQtEnc(in.encoding, swap);
        
        return swap ?
            QImage(in.data.data(), in.width, in.height, in.step, f).rgbSwapped() :
            QImage(in.data.data(), in.width, in.height, in.step, f);
    }

    QImage rosImgToQimg(const sensor_msgs::ImageConstPtr& in)
    {   
        return rosImgToQimg(*in);
    }
    
    QPixmap rosImgToQpixmap(const sensor_msgs::Image& in)
    {
        return QPixmap::fromImage(rosImgToQimg(in));
    }
    
    QPixmap rosImgToQpixmap(const sensor_msgs::ImageConstPtr& in)
    {
        return QPixmap::fromImage(rosImgToQimg(in));
    }

    cv::Mat qImgToMat(const QImage& in)
    {   
        QImage temp = in.rgbSwapped();
        int format = (temp.format() == QImage::Format_Grayscale8) ? CV_8UC1 : CV_8UC3;
        return cv::Mat(temp.height(), temp.width(), format, (uchar *)temp.bits(), temp.bytesPerLine());
    }

    cv::Mat rosImgToMat(const sensor_msgs::Image& in)
    {
        int format = (in.encoding == "mono8") ? CV_8UC1 : CV_8UC3;
        return cv::Mat(in.height, in.width, format, (uchar *)in.data.data(), in.step);
    }

    void qImgToRosImg(const QImage& in, sensor_msgs::Image& out)
    {   
        QImage temp = in;//.rgbSwapped();
        std::string format = (temp.format() == QImage::Format_Grayscale8) ? "mono8" : "bgr8";
        sensor_msgs::fillImage(out, format, temp.height(), temp.width(), temp.bytesPerLine(), temp.bits());
    }

    sensor_msgs::ImagePtr matToRosImg(const cv::Mat& in)
    {
        std::string format = (in.channels() == 1) ? "mono8" : "rgb8";
        return cv_bridge::CvImage(std_msgs::Header(), format, in).toImageMsg();
    }
    
//    void qPixmapToRosImg(const QPixmap& in, sensor_msgs::Image& out)
//    {
//        out.data = in.data_ptr().data();
//    }

    int rosEncToQtEnc(std::string enc, bool& swap)
    {
        swap = false;
        if(enc == "mono8")
            return QImage::Format_Grayscale8;
        if(enc == "rgb8")
            return QImage::Format_RGB888;
        if(enc == "bgr8")
        {
            swap = true;
            return QImage::Format_RGB888;
        }
        
        return QImage::Format_Invalid;
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
    
} //namespace gcs
