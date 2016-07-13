/*
 * File:   UnansweredQueries.cpp
 * Author: serl
 *
 * Created on July 11, 2016, 2:24 PM
 */

#include "rqt_gcs/unanswered_queries.h"

namespace rqt_gcs
{

namespace fs = boost::filesystem;
    
UnansweredQueries::UnansweredQueries(SimpleGCS * sgcs) :
gcs(sgcs)
{
    widget.setupUi(this);
    
}

UnansweredQueries::~UnansweredQueries()
{
    gcs = nullptr;
}

void UnansweredQueries::addQuery(cv::Mat& img)
{
    //retrieve Query msg for door image
    QImage image(img.data,
                 img.cols,
                 img.rows, 
                 QImage::Format_RGB888);

    //create the widget
    QWidget * pmWidget = new QWidget();
    Ui::PictureMsgWidget pmUiWidget;
    pmUiWidget.setupUi(pmWidget);

    QWidget * imgWidget = new QWidget();
    Ui::ImageViewWidget imgUiWidget;
    imgUiWidget.setupUi(imgWidget);

    // take care of the image
    imgUiWidget.image_frame->setImage(image);
    widget.verticalLayout->addWidget(pmWidget);
}

}