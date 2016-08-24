
#include "rqt_gcs/access_points_menu.h"

namespace rqt_gcs
{
    
AccessPointsMenu::AccessPointsMenu() :
    mapper(new QSignalMapper(this))
{
    widget_.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
}

AccessPointsMenu::~AccessPointsMenu()
{
    clearAccessPoints();
    emit deleted();
}

void AccessPointsMenu::setUavTitle(int uav_id)
{
    QString s = "UAV " % QString::number(uav_id);
    widget_.lbl_uav->setText(s);
}

void AccessPointsMenu::clearAccessPoints()
{
    //clear out old list of Access points widgets
    int size = ap_widgets.size();
    for(int i = 0; i < size; i--)
    {
        widget_.layout_access_points->removeWidget(ap_widgets[i]);
        delete ap_widgets[i];
    }
    ap_widgets.clear();

    num_access_points_last = 0;
}

void AccessPointsMenu::updateAccessPoints(SimpleControl* uav)
{
    if(!this->isVisible())
        return;

    //retreive access points
    std::vector<AccessPoint> * ap_vec = uav->GetRefAccessPoints();

    //Get our new number of Access points
    int apv_size = ap_vec->size();
    int new_access_points = apv_size - num_access_points_last;

    for(int i = 0; i < new_access_points; i++)
    {
        //retrieve access point
        AccessPoint accessPoint = ap_vec->at(i + num_access_points_last);
        sensor_msgs::Image acImage = accessPoint.GetImage();

        QImage image(acImage.data.data(), acImage.width, acImage.height,
                     acImage.step, QImage::Format_RGB888);

        QWidget * apWidget = new QWidget();
        Ui::AccessPointStatsWidget apUiWidget;
        apUiWidget.setupUi(apWidget);

        QWidget * imageWidget = new QWidget();
        Ui::ImageViewWidget imageUiWidget;
        imageUiWidget.setupUi(imageWidget);

         //image
        imageUiWidget.image_frame->setImage(image);
        apUiWidget.PictureLayout->addWidget(imageWidget);

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

void AccessPointsMenu::deleteAccessPoint(QWidget* w)
{
    int deleteIndex = widget_.layout_access_points->indexOf(w);
    widget_.layout_access_points->removeWidget(w);
    ap_widgets.remove(deleteIndex);
    delete w;
    num_access_points_last--;
}


void AccessPointsMenu::saveUavAccessPoints(SimpleControl* uav, QString& ap_type)
{
//    QString path = image_root_dir_ % "/access_points/" % ap_type;
//    path.append("/uav_" + QString::number(uav->id));
//    std::vector<AccessPoint> * ap_vector = uav->GetRefAccessPoints();
//    for(int i = 0; i < ap_vector->size(); i++)
//    {
//        QString file = "img_" + QString::number(i) + ".jpg";
//
//        AccessPoint ap = ap_vector->at(i);
//        sensor_msgs::Image ros_image = ap.GetImage();
//        QImage image(ros_image.data.data(), ros_image.width, ros_image.height,
//                     ros_image.step, QImage::Format_RGB888);
//        saveImage(path, file, image);
//    }
}

bool AccessPointsMenu::saveImage(QString& path, QString& file, QImage* image)
{
    QDir dir(path);
    if(!dir.exists())
        dir.mkdir(path);

    QString full_path = dir.canonicalPath().append("/" % file);
    return image->save(full_path, "jpg", -1);
}

}