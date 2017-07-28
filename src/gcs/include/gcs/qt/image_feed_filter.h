
/* 
 * File:   ImageFeedFilter.h
 * Author: serl
 *
 * Created on July 28, 2017, 10:15 AM
 */

#ifndef IMAGEFEEDFILTER_H
#define IMAGEFEEDFILTER_H

#include <QObject>

namespace gcs
{

    class GCSMainWindow;
    
class ImageFeedFilter : public QObject
{
    Q_OBJECT
public:
    ImageFeedFilter(GCSMainWindow * main_window, QObject * parent = nullptr);
    
   
protected:
    bool eventFilter(QObject *obj, QEvent *event);
    
private:
    GCSMainWindow * _main_window;
};

}
#endif /* IMAGEFEEDFILTER_H */

