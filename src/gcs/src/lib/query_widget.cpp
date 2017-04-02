/*
 * File:   query_widget.cpp
 * Author: n8
 *
 * Created on September 26, 2016, 7:20 PM
 */

#include <QPixmap>

#include <gcs/qt/query_widget.h>

namespace gcs
{

QueryWidget::QueryWidget()
{
    widget.setupUi(this);
}

QueryWidget::~QueryWidget()
{
}

void QueryWidget::SetImage(const QPixmap& img)
{
    int w = widget.image_frame->width();
    int h = widget.image_frame->height();
    widget.image_frame->setPixmap(img.scaled(w, h, Qt::KeepAspectRatio));
}

const QPushButton * QueryWidget::YesButton()
{
    return widget.yesButton;
}

const QPushButton * QueryWidget::RejectButton()
{
    return widget.rejectButton;
}

}