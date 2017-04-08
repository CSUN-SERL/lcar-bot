
/* 
 * File:   map_widget_base.cpp
 * Author: n8
 * 
 * Created on April 7, 2017, 11:55 PM
 */

#include <gcs/qt/map_widget.h>
#include <gcs/qt/gcs_main_window.h>

namespace gcs
{

MapWidget::MapWidget(GCSMainWindow * mw) :
QWidget(mw),
main_window(mw)
{
    QVBoxLayout * layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    
    QWidget::setLayout(layout);
    QWidget::setContentsMargins(0, 0, 0, 0); 
    
    QSizePolicy size_policy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    size_policy.setHorizontalStretch(1);
    size_policy.setVerticalStretch(1);
    
    QWidget::setSizePolicy(size_policy);
}

MapWidget::~MapWidget() 
{
}

}