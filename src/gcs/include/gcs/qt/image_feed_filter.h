
/* 
 * File:   ImageFeedFilter.h
 * Author: serl
 *
 * Created on July 28, 2017, 10:15 AM
 */

#ifndef IMAGEFEEDFILTER_H
#define IMAGEFEEDFILTER_H

#include <memory>

#include <QObject>

namespace gcs
{

class GCSMainWindow;
class Building;
    
class ImageFeedFilter : public QObject
{
    Q_OBJECT
public:
    ImageFeedFilter(GCSMainWindow * main_window, QObject * parent = nullptr);
    
    void setCurrentBuilding(const std::shared_ptr<Building>& building);
   
protected:
    bool eventFilter(QObject *obj, QEvent *event);
    
private:
    GCSMainWindow * _main_window;
    std::shared_ptr<Building> _cur_building;
};

}
#endif /* IMAGEFEEDFILTER_H */

