
#ifndef IMAGEHANDLER_H
#define IMAGEHANDLER_H

#include <QDir>
#include <QStringBuilder>
#include <QImage>

#include <opencv/cv.h>

namespace rqt_gcs
{

class ImageHandler
{
    
public:
    ImageHandler();
    virtual ~ImageHandler();
    static bool saveImage(QString& path, QString& file, QImage* image);
    void cvMatToQimage(const cv::Mat& in, QImage& out);
    void qImageToCvMat(const QImage& in, cv::Mat& out);
    
private:
    

};

}

#endif /* IMAGEHANDLER_H */

