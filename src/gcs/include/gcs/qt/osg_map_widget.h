
/* 
 * File:   osg_map_widget.h
 * Author: n8
 *
 * Created on April 7, 2017, 11:13 PM
 */

#ifndef OSG_MAP_WIDGET_H
#define OSG_MAP_WIDGET_H

#include <QStatusBar>

#include <gcs/qt/map_widget_base.h>

#include <osgEarth/GeoData>
#include <osgEarthUtil/MouseCoordsTool>
#include <osgEarth/MapNode>
#include <osg/View>

namespace osgEarth
{
    namespace QtGui
    {
        class ViewerWidget;
    }
}

namespace osgViewer
{
    class Viewer;
}

using namespace osg;
using namespace osgViewer;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;


namespace gcs
{
    
class OsgMapWidget : public MapWidgetBase
{
    Q_OBJECT
public:
    OsgMapWidget(GCSMainWindow * mw);
    virtual ~OsgMapWidget();
    virtual void load() Q_DECL_OVERRIDE;
private:    
    ViewerWidget * osg_map;
    Viewer * viewer;
};

struct PrintCoordsToStatusBar : public MouseCoordsTool::Callback
{
public:
    PrintCoordsToStatusBar(QStatusBar* sb) : _sb(sb) { }

    void set(const osgEarth::GeoPoint& p, osg::View* view, osgEarth::MapNode* mapNode)
    {
        std::string str = osgEarth::Stringify() << p.y() << ", " << p.x();
        _sb->showMessage( QString(str.c_str()) );
    }

    void reset(osg::View* view, osgEarth::MapNode* mapNode)
    {
        _sb->showMessage( QString("out of range") );
    }

    QStatusBar* _sb;
};

}

#endif /* OSG_MAP_WIDGET_H */

