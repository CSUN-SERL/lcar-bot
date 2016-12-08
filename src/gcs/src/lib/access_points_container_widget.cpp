
#include <QStringBuilder>

#include "qt/access_points_container_widget.h"
#include "qt/access_point_widget.h"
#include "qt/ui_adapter.h"
#include "util/image.h"

namespace gcs
{
    
AccessPointsContainerWidget::AccessPointsContainerWidget(QString& img_dir) :
    image_root_dir(img_dir),
    timer(new QTimer(this))
{
    widget.setupUi(this);
    this->setAttribute(Qt::WA_DeleteOnClose);
    
    connect(UIAdapter::Instance(), &UIAdapter::SetImageRootDir,
            this, &AccessPointsContainerWidget::OnUpdateImageRootDir);
    
    connect(timer, &QTimer::timeout, 
            this, &AccessPointsContainerWidget::UpdateAccessPoints);
    timer->start(0);
}

AccessPointsContainerWidget::~AccessPointsContainerWidget()
{
    this->ClearAccessPoints();
    ap_vec = nullptr;
}

void AccessPointsContainerWidget::SetUAVAccessPointsAndId(std::vector<lcar_msgs::AccessPointStampedPtr> * ap_vec, int v_id)
{
    this->ap_vec = ap_vec;
    if(ap_vec != nullptr)
        widget.lbl_uav->setText("UAV " % QString::number(v_id));
    else
        widget.lbl_uav->setText("NO UAVS");
    
    this->ClearAccessPoints();
}

void AccessPointsContainerWidget::ClearAccessPoints()
{
    //clear out old list of Access points widgets
    for(int i = widget.layout_access_points->count() - 1 ; i >= 0; i--)
        delete widget.layout_access_points->itemAt(i)->widget();

    num_access_points_last = 0;
}

void AccessPointsContainerWidget::UpdateAccessPoints()
{
    if(!this->isVisible() || ap_vec == nullptr)
        return;

    //Get our new number of Access points
    int apv_size = ap_vec->size();

    for(int i = num_access_points_last; i < apv_size; i++)
    {
        //retrieve access point
        lcar_msgs::AccessPointStampedPtr accessPoint = ap_vec->at(i);
        QPixmap image = img::rosImgToQpixmap(accessPoint->ap.query.img_framed);

        AccessPointWidget *ap_widget = new AccessPointWidget(this);
        
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

        widget.layout_access_points->addWidget(ap_widget);
    }
    num_access_points_last = apv_size;
}

void AccessPointsContainerWidget::OnDeleteAccessPoint(QWidget *w)
{
    int index = widget.layout_access_points->indexOf(w);
    delete w;
    
    ap_vec->erase(ap_vec->begin() + index);
    num_access_points_last--;
}

void AccessPointsContainerWidget::OnUpdateImageRootDir(QString new_dir)
{
    this->image_root_dir = new_dir;
}

void AccessPointsContainerWidget::SaveUavAccessPoints(std::vector<lcar_msgs::AccessPointStampedPtr> *ap_vector, int id, QString ap_type, QString img_dir)
{
    QString path = img_dir % "/access_points/" % ap_type;
    path.append("/uav_" + QString::number(id));
    for(int i = 0; i < ap_vector->size(); i++)
    {
        QString file = "img_" + QString::number(i) + ".jpg";
        lcar_msgs::AccessPointStampedPtr ap = ap_vector->at(i);
        img::saveImage(path, file, img::rosImgToQimg(ap->ap.query.img));
    }
}


}