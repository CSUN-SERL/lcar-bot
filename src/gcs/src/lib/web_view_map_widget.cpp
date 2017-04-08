
/* 
 * File:   WebViewMapWidget.cpp
 * Author: n8
 * 
 * Created on April 8, 2017, 12:24 AM
 */

#include <QtWebKitWidgets/QWebView>

#include <ros/package.h>

#include <gcs/qt/web_view_map_widget.h>
#include <gcs/qt/gcs_main_window.h>

namespace gcs
{

WebViewMapWidget::WebViewMapWidget(GCSMainWindow * mw) :
MapWidget(mw)
{
}

WebViewMapWidget::~WebViewMapWidget()
{
}

void WebViewMapWidget::load(const QString& file)
{   
    QString map_url;
    if(file.isNull() || file.isEmpty())
    {
        QString s(ros::package::getPath("gcs").c_str());
        map_url = QString("file://%1/map/uavmap.html").arg(s);
    }
    else
    {
        map_url = file;   
    }
    
    if(!web_view)
    {
        web_view = new QWebView(this);
        
        QWidget::layout()->addWidget(web_view);
    }

    web_view->load(QUrl(map_url));
}

}