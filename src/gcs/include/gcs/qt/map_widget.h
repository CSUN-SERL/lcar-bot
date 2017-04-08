
/* 
 * File:   map_widget_base.h
 * Author: n8
 *
 * Created on April 7, 2017, 11:55 PM
 */

#ifndef MAPWIDGETBASE_H
#define MAPWIDGETBASE_H

#include <QWidget>

namespace gcs
{

class GCSMainWindow;    
    
class MapWidgetBase : public QWidget
{
    Q_OBJECT
public:
    MapWidgetBase(GCSMainWindow * mw);
    virtual ~MapWidgetBase();
    
    virtual void load() = 0;

protected:
    GCSMainWindow * main_window;
    
private:

};

}
#endif /* MAPWIDGETBASE_H */

