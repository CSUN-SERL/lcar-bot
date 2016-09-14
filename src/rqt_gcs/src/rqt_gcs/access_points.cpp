
#include "rqt_gcs/access_points.h"
#include "rqt_gcs/image_utility.h"
#include <QStringBuilder>
#include "ui_AccessPointStats.h"

namespace rqt_gcs
{
    
AccessPoints::AccessPoints() :
    mapper(new QSignalMapper(this)),
    timer(new QTimer(this))
{
    widget_.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    connect(timer, &QTimer::timeout, 
            this, &AccessPoints::updateAccessPoints);
    timer->start(0);
}

AccessPoints::~AccessPoints()
{
    clearAccessPoints();
    uav = nullptr;
}

void AccessPoints::setUav(UAVControl* uav)
{
    this->uav = uav;
    if(uav != nullptr)
        widget_.lbl_uav->setText("UAV " % QString::number(uav->id));
    else
        widget_.lbl_uav->setText("NO UAVS");
    
    clearAccessPoints();
}

void AccessPoints::clearAccessPoints()
{
    //clear out old list of Access points widgets
    int size = ap_widgets.size();
    for(int i = 0; i < size; i++)
    {
        widget_.layout_access_points->removeWidget(ap_widgets[i]);
        delete ap_widgets[i];
    }
    ap_widgets.clear();
    num_access_points_last = 0;
}

void AccessPoints::updateAccessPoints()
{
    if(!this->isVisible() || uav == nullptr)
        return ;

    //retreive access points
    std::vector<AccessPoint> * ap_vec = uav->GetRefAccessPoints();

    //Get our new number of Access points
    int apv_size = ap_vec->size();
    int new_access_points = apv_size - num_access_points_last;

    for(int i = 0; i < new_access_points; i++)
    {
        //retrieve access point
        AccessPoint accessPoint = ap_vec->at(i + num_access_points_last);
        sensor_msgs::Image ap_img = accessPoint.GetImage();

        QImage image = image_util::rosImgToQimg(ap_img);

        QWidget * apWidget = new QWidget();
        Ui::AccessPointStatsWidget apUiWidget;
        apUiWidget.setupUi(apWidget);

        image = image.scaled(apWidget->width(), apWidget->height(), 
                             Qt::AspectRatioMode::KeepAspectRatio);
        apUiWidget.image_frame->setPixmap(QPixmap::fromImage(image));

        //add widget to the list
        ap_widgets.push_back(apWidget);
        int index = ap_widgets.size() - 1;

        //access point name
        QString ap_id, ap_data;
        ap_id.setNum(i + num_access_points_last + 1);
        ap_data = "Access Point " + ap_id;
        apUiWidget.buildingNameLine->setText(ap_data);

        //altitude
        ap_data.setNum(accessPoint.GetAltitude().data, 'f', 2);
        apUiWidget.altitudeLineEdit->setText(ap_data);

        //heading
        ap_data.setNum(accessPoint.GetHeading().data, 'f', 2);
        apUiWidget.compassHeadingLineEdit->setText(ap_data);

        //location longitude
        ap_data.setNum(accessPoint.GetLocation().longitude, 'f', 2);
        apUiWidget.longitudeLineEdit->setText(ap_data);

        //location latitude
        ap_data.setNum(accessPoint.GetLocation().latitude, 'f', 2);
        apUiWidget.latitudeLineEdit->setText(ap_data);

        //time
        ap_data.setNum(accessPoint.GetTime().toSec(), 'f', 6);
        apUiWidget.captureTimeLineEdit->setText(ap_data);

        //map signal to the delete button
        mapper->setMapping(apUiWidget.deleteAccessPointButton,
                                   ap_widgets[index]);

        connect(apUiWidget.deleteAccessPointButton, SIGNAL(clicked()),
                mapper, SLOT(map()));

        //finally, add it to the gui
        widget_.layout_access_points->addWidget(ap_widgets[index]);
    }
    num_access_points_last = apv_size;
}

void AccessPoints::deleteAccessPoint(QWidget* w)
{
    int deleteIndex = widget_.layout_access_points->indexOf(w);
    widget_.layout_access_points->removeWidget(w);
    ap_widgets.remove(deleteIndex);
    delete w;
    auto vector = uav->GetRefAccessPoints();
    vector->erase(vector->begin()+deleteIndex);
    num_access_points_last--;
}


void AccessPoints::saveUavAccessPoints(UAVControl* uav, QString ap_type)
{
    QString path = image_util::image_root_dir_ % "/access_points/" % ap_type;
    path.append("/uav_" + QString::number(uav->id));
    std::vector<AccessPoint> * ap_vector = uav->GetRefAccessPoints();
    for(int i = 0; i < ap_vector->size(); i++)
    {
        QString file = "img_" + QString::number(i) + ".jpg";

        AccessPoint ap = ap_vector->at(i);
        sensor_msgs::Image ros_image = ap.GetImage();
        QImage image(ros_image.data.data(), ros_image.width, ros_image.height,
                     ros_image.step, QImage::Format_RGB888);
        image_util::saveImage(path, file, image);
    }
}


}