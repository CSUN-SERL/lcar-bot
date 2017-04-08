
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


namespace gcs
{

OsgMapWidget::OsgMapWidget(GCSMainWindow * mw) :
MapWidget(mw),
osg_map(nullptr),
viewer(nullptr)
{
}

OsgMapWidget::~OsgMapWidget() 
{
}

void OsgMapWidget::load(const QString& file)
{
    ref_ptr<Node> node;
    if(file.isNull() || file.isEmpty())
    {
        node = osgDB::readFile<Node>(file.toStdString());
    }
    else
    {
        node = osgDB::readFile<Node>("aero-chart-arcgis.earth");
    }
    
    if(!node)
        return;
    
    if(osg_map)
    {
        //todo handle reloading better
        delete osg_map;
        viewer = nullptr;
    }

    osg_map = new ViewerWidget(node);
    viewer = static_cast<Viewer*>(osg_map->getViewer());
    QWidget::layout()->addWidget(osg_map);
    
    MapNode * map_node = MapNode::get(node);
    MouseCoordsTool * tool = new MouseCoordsTool(map_node);
    PrintCoordsToStatusBar* print_status = new PrintCoordsToStatusBar(main_window->statusBar());
    tool->addCallback(print_status);
   
    viewer->addEventHandler(tool);
}

}