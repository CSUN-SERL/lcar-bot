
/* 
 * File:   map_widget_base.cpp
 * Author: n8
 * 
 * Created on April 7, 2017, 11:55 PM
 */

#include <gcs/qt/map_widget_base.h>
#include <gcs/qt/gcs_main_window.h>

namespace gcs
{

MapWidgetBase::MapWidgetBase(GCSMainWindow * mw) :
QWidget(mw),
main_window(mw)
{
}

MapWidgetBase::~MapWidgetBase() 
{
}

}