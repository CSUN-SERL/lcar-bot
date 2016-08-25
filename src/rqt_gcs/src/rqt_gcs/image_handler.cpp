
#include "rqt_gcs/image_handler.h"

namespace rqt_gcs
{

ImageHandler::ImageHandler(){ }


ImageHandler::~ImageHandler(){ }

static bool ImageHandler::saveImage(QString& path, QString& file, QImage* image)
{
    QDir dir(path);
    if(!dir.exists())
        dir.mkdir(path);

    QString full_path = dir.canonicalPath().append("/" % file);
    return image->save(full_path, "jpg", -1);
}

void ImageHandler::cvMatToQimage(const cv::Mat& in, QImage& out)
{
    out = QImage(in.data, in.cols, in.rows, in.step, QImage::Format_RGB888);
}




}
