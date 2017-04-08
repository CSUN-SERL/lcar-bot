
/* 
 * File:   map_widget_base.h
 * Author: n8
 *
 * Created on April 7, 2017, 11:55 PM
 */

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>

namespace gcs
{

class GCSMainWindow;    
    
class MapWidget : public QWidget
{
    Q_OBJECT
public:
    MapWidget(GCSMainWindow * mw);
    virtual ~MapWidget();
    
    virtual void load(const QString& file = QString()) = 0;

protected:
    GCSMainWindow * main_window;
    
private:

};

}
#endif /* MAPWIDGET_H */

