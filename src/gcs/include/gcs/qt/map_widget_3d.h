
/* 
 * File:   map_widget_3d.h
 * Author: n8
 *
 * Created on June 30, 2017, 5:11 PM
 */

#ifndef MAP_WIDGET_3D_H
#define MAP_WIDGET_3D_H

#include <QWidget>

class QVector3D;

namespace Qt3DExtras
{
class Qt3DWindow;
}

namespace Qt3DCore
{
    class QEntity;
}

namespace gcs
{
    class VehicleManager;
}
    
class MapWidget3D : public QWidget
{
    Q_OBJECT
public:
    MapWidget3D(QWidget * parent = nullptr);
    virtual ~MapWidget3D();
    
    void setVehicleManager(gcs::VehicleManager * vm);
    
private:
    Q_DISABLE_COPY(MapWidget3D)
    
    void setupUi();
    
    void createDefaultScene();
    
    void createFloor();
    void createCameraController();
    void createLighting(const QVector3D& pos, float instensity);
    void createBuilding(const QVector3D& pos, float size);
            
private:
    gcs::VehicleManager * _vm;
    
    Qt3DExtras::Qt3DWindow * _view;
    Qt3DCore::QEntity * _root;
    Qt3DCore::QEntity * _plane;
};


#endif /* MAP_WIDGET_3D_H */

