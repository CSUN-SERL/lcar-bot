
#include <QStringBuilder>

#include "rqt_gcs/access_points.h"
#include "util/image.h"

#include "ui_AccessPointStats.h"

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
    std::vector<AccessPoint> * ap_vec = uav->GetRefAccessPoints();

    //Get our new number of Access points
    int apv_size = ap_vec->size();

    for(int i = num_access_points_last; i < apv_size; i++)
    {
        //retrieve access point
        AccessPoint accessPoint = ap_vec->at(i);
        sensor_msgs::ImagePtr ap_img = accessPoint.GetImage();

        QPixmap image = img::rosImgToQpixmap(ap_img);

        QWidget * ap_widget = new QWidget(this);
        Ui::AccessPointStatsWidget ui;
        ui.setupUi(ap_widget);

        int w = ui.image_frame->width();
        int h = ui.image_frame->height();
        image = image.scaled(w, h, Qt::AspectRatioMode::KeepAspectRatio);
        ui.image_frame->setPixmap(image);

        //access point name
        QString ap_id, ap_data;
        ap_id.setNum(i);
        ap_data = "Access Point " + ap_id;
        ui.buildingNameLine->setText(ap_data);

        //altitude
        ap_data.setNum(accessPoint.GetAltitude().data, 'f', 2);
        ui.altitudeLineEdit->setText(ap_data);

        //heading
        ap_data.setNum(accessPoint.GetHeading().data, 'f', 2);
        ui.compassHeadingLineEdit->setText(ap_data);

        //location longitude
        ap_data.setNum(accessPoint.GetLocation().longitude, 'f', 2);
        ui.longitudeLineEdit->setText(ap_data);

        //location latitude
        ap_data.setNum(accessPoint.GetLocation().latitude, 'f', 2);
        ui.latitudeLineEdit->setText(ap_data);

        //time
        ap_data.setNum(accessPoint.GetTime().toSec(), 'f', 6);
        ui.captureTimeLineEdit->setText(ap_data);

        connect(ui.deleteAccessPointButton, &QPushButton::clicked,
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
    std::vector<AccessPoint>* vector = uav->GetRefAccessPoints();
    vector->erase(vector->begin()+deleteIndex);
    num_access_points_last--;
}


void AccessPoints::SaveUavAccessPoints(UAVControl* uav, QString ap_type)
{
    QString path = img::image_root_dir_ % "/access_points/" % ap_type;
    path.append("/uav_" + QString::number(uav->id));
    std::vector<AccessPoint> * ap_vector = uav->GetRefAccessPoints();
    for(int i = 0; i < ap_vector->size(); i++)
    {
        QString file = "img_" + QString::number(i) + ".jpg";

        AccessPoint ap = ap_vector->at(i);
        sensor_msgs::ImagePtr ros_image = ap.GetImage();
        QImage image(ros_image->data.data(), ros_image->width, ros_image->height,
                     ros_image->step, QImage::Format_RGB888);
        img::saveImage(path, file, image);
    }
}


}