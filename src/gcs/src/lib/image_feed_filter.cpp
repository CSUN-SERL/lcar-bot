
/* 
 * File:   ImageFeedFilter.cpp
 * Author: serl
 * 
 * Created on July 28, 2017, 10:15 AM
 */

#include <gcs/qt/image_feed_filter.h>
#include <gcs/qt/gcs_main_window.h>

namespace gcs
{

ImageFeedFilter::ImageFeedFilter(GCSMainWindow * main_window, QObject * parent) :
QObject(parent),
 _main_window(main_window)
{
}

bool ImageFeedFilter::eventFilter(QObject *obj, QEvent *event)
{
    if(event->type() == QEvent::KeyPress)
        _main_window->setImageFeedVisible(true);
    else if(event->type() == QEvent::KeyRelease)
        _main_window->setImageFeedVisible(false);
    
    return false;
}

}