
/* 
 * File:   WebViewMapWidget.h
 * Author: n8
 *
 * Created on April 8, 2017, 12:24 AM
 */

#ifndef WEBVIEWMAPWIDGET_H
#define WEBVIEWMAPWIDGET_H

#include <QPointer>

#include <gcs/qt/map_widget.h>

class QWebView;

namespace gcs
{

class GCSMainWindow;
    
class WebViewMapWidget : public MapWidget
{
    Q_OBJECT
public:
    WebViewMapWidget(GCSMainWindow * mw);
    virtual ~WebViewMapWidget();
    
    virtual void load(const QString& file = QString()) Q_DECL_OVERRIDE;
    
private:
    QPointer<QWebView> web_view;
};

}
#endif /* WEBVIEWMAPWIDGET_H */

