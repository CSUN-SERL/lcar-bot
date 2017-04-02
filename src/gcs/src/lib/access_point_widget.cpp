
/*
 * File:   access_point_widget.cpp
 * Author: n8
 *
 * Created on October 6, 2016, 2:54 PM
 */

#include <QPixmap>

#include <gcs/qt/access_point_widget.h>

namespace gcs
{

AccessPointWidget::AccessPointWidget(QWidget *parent):
MyQWidget(parent)
{
    widget.setupUi(this);
}

AccessPointWidget::~AccessPointWidget()
{
}

void AccessPointWidget::SetName(QString name)
{
    widget.buildingNameLine->setText(name);
}

void AccessPointWidget::SetLatitude(double lat)
{
    widget.latitudeLineEdit->setText(QString::number(lat, 'f' ,6));
}

void AccessPointWidget::SetLongitude(double lng)
{
    widget.longitudeLineEdit->setText(QString::number(lng, 'f', 6));
}

void AccessPointWidget::SetHeading(double heading)
{
    widget.compassHeadingLineEdit->setText(QString::number(heading, 'f', 3));
}

void AccessPointWidget::SetAltitude(double altitude)
{
    widget.altitudeLineEdit->setText(QString::number(altitude, 'f', 3));
}

void AccessPointWidget::SetCaptureTime(double time)
{
    widget.captureTimeLineEdit->setText(QString::number(time, 'f', 6));
}

void AccessPointWidget::SetImage(QPixmap& img)
{
    int w = widget.image_frame->width();
    int h = widget.image_frame->height();
    widget.image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

const QPushButton * AccessPointWidget::Button()
{
    return widget.deleteAccessPointButton;
}

}