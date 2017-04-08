
/* 
 * File:   osg_map_widget.cpp
 * Author: n8
 * 
 * Created on April 7, 2017, 11:13 PM
 */

#include <QVBoxLayout>


#include <gcs/qt/osg_map_widget.h>
#include <gcs/qt/gcs_main_window.h>


#include <osgEarthQt/ViewerWidget>
#include <osgDB/ReadFile>

using namespace osg;
using namespace osgViewer;
using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::QtGui;

namespace gcs
{

OsgMapWidget::OsgMapWidget(GCSMainWindow * mw) :
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

OsgMapWidget::~OsgMapWidget() 
{
}

void OsgMapWidget::load()
{
    ref_ptr<Node> node = osgDB::readFile<Node>("aero-chart-arcgis.earth");    
    
    if(!node)
        return;
    
    osg_map = new ViewerWidget(node);
    QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    sizePolicy.setHorizontalStretch(1);
    sizePolicy.setVerticalStretch(1);
    osg_map->setSizePolicy(sizePolicy);
    
    MapNode * map_node = MapNode::get(node);
    MouseCoordsTool * tool = new MouseCoordsTool(map_node);
    PrintCoordsToStatusBar* print_status = new PrintCoordsToStatusBar(main_window->statusBar());
    tool->addCallback(print_status);
    
    viewer = static_cast<Viewer*>(osg_map->getViewer());
    viewer->addEventHandler(tool);
    
    QWidget::layout()->addWidget(osg_map);
}

}