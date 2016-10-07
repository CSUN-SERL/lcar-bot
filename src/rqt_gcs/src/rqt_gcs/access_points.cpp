
#include <QStringBuilder>

#include "rqt_gcs/all_access_points_widget.h"
#include "rqt_gcs/access_point_widget.h"
#include "util/image.h"
#include "ui_AllAccessPointsWidget.h"

namespace rqt_gcs
{
    
AccessPoints::AccessPoints() :
    timer(new QTimer(this))
{
    widget.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    connect(timer, &QTimer::timeout, 
            this, &AccessPoints::UpdateAccessPoints);
    timer->start(0);
}

AccessPoints::~AccessPoints()
{
    ClearAccessPoints();
    uav = nullptr;
}

void AccessPoints::SetUAV(UAVControl* uav)
{
    this->uav = uav;
    if(uav != nullptr)
        widget.lbl_uav->setText("UAV " % QString::number(uav->id));
    else
        widget.lbl_uav->setText("NO UAVS");
    
    this->ClearAccessPoints();
}

void AccessPoints::ClearAccessPoints()
{
    //clear out old list of Access points widgets
    for(int i = widget.layout_access_points->count() -1 ; i >= 0; i--)
        delete widget.layout_access_points->itemAt(i)->widget();

    num_access_points_last = 0;
}

void AccessPoints::UpdateAccessPoints()
{
    if(!this->isVisible() || uav == nullptr)
        return;

    //retreive access points
    std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vec = uav->GetRefAccessPoints();

    //Get our new number of Access points
    int apv_size = ap_vec->size();

    for(int i = num_access_points_last; i < apv_size; i++)
    {
        //retrieve access point
        lcar_msgs::AccessPointStampedPtr accessPoint = ap_vec->at(i);
        sensor_msgs::Image* ap_img = &accessPoint->ap.query.img_framed;

        QPixmap image = img::rosImgToQpixmap(*ap_img);

        AccessPointWidget * ap_widget = new AccessPointWidget(this);
        
        ap_widget->SetImage(image);
        
        //access point name
        ap_widget->SetName("Building " % QString::number(i));
        
        //altitude
        ap_widget->SetAltitude((double)accessPoint->ap.altitude);

        //heading
        ap_widget->SetHeading((double)accessPoint->ap.compass_heading);

        //location longitude
        ap_widget->SetLongitude((double)accessPoint->ap.location.longitude);

        //location latitude
        ap_widget->SetLatitude((double)accessPoint->ap.location.latitude);

        //time
        ap_widget->SetCaptureTime((double)accessPoint->header.stamp.toSec());
 

        connect(ap_widget->Button(), &QPushButton::clicked,
                this, [=](){ OnDeleteAccessPoint(ap_widget); } );

        //finally, add it to the gui
        widget.layout_access_points->addWidget(ap_widget);
    }
    num_access_points_last = apv_size;
}

void AccessPoints::OnDeleteAccessPoint(QWidget* w)
{
    int deleteIndex = widget.layout_access_points->indexOf(w);
    widget.layout_access_points->removeWidget(w);
    delete w;
    std::vector<lcar_msgs::AccessPointStampedPtr>* vector = uav->GetRefAccessPoints();
    vector->erase(vector->begin()+deleteIndex);
    num_access_points_last--;
}


void AccessPoints::SaveUavAccessPoints(UAVControl* uav, QString ap_type)
{
    QString path = img::image_root_dir_ % "/access_points/" % ap_type;
    path.append("/uav_" + QString::number(uav->id));
    std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vector = uav->GetRefAccessPoints();
    for(int i = 0; i < ap_vector->size(); i++)
    {
        QString file = "img_" + QString::number(i) + ".jpg";

        lcar_msgs::AccessPointStampedPtr ap = ap_vector->at(i);
        sensor_msgs::Image* ros_image = &ap->ap.query.img;
        QImage image(ros_image->data.data(), ros_image->width, ros_image->height,
                     ros_image->step, QImage::Format_RGB888);
        img::saveImage(path, file, image);
    }
}


}