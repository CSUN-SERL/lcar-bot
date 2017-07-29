
/* 
 * File:   map_widget_3d.h
 * Author: n8
 *
 * Created on June 30, 2017, 5:11 PM
 */

#ifndef MAP_WIDGET_3D_H
#define MAP_WIDGET_3D_H

#include <memory>

#include <QWidget>
#include <QMap>

#include <vehicle/data_types.h>
#include <vehicle/position.h>

#include <gcs/qt/gcs_main_window.h>
#include <gcs/qt/image_feed_filter.h>

class QFrame;
class QVector3D;

namespace Qt3DRender
{
    class QMesh;
    class QMaterial;
}

namespace Qt3DExtras
{
    class Qt3DWindow;
}

namespace Qt3DCore
{
    class QEntity;
    class QTransform;
}

typedef Qt3DCore::QTransform Transform;

namespace gcs
{
    class VehicleManager;
    class VehicleControl;
    class ImageFeedFilter;
    class Building;
}

class Window3D;

class MapWidget3D : public QWidget
{
    Q_OBJECT
    
private:
    struct Vehicle3D
    {        
        Qt3DCore::QEntity * _entity;
        Qt3DRender::QMesh *_mesh;
        Qt3DRender::QMaterial * _material;
        Transform * _transform;
        
        gcs::VehicleControl * _vehicle;
        
        int _cur_waypoint;
        
        void update();
    };
    
public:
    MapWidget3D(QWidget * parent = nullptr);
    virtual ~MapWidget3D();
    
    void setImageFeedFilter(gcs::ImageFeedFilter * filter);
    void setVehicleManager(gcs::VehicleManager * vm);
    void setUpdateTimer(QTimer * timer);
    
private slots:
    void update();
    void vehicleAdded(int v_id);
    
private:
    Q_DISABLE_COPY(MapWidget3D)
            
    void createDefaultScene();
    
    void createFloor();
    void createCameraController();
    void createLighting(const QVector3D& pos, float instensity);
    void createBuilding(const QVector3D& pos, float size);
    Vehicle3D * createVehicle(int vehicle_type);
    
    
    void connectToUiAdapter();
    void setupUi();
    
private:
    gcs::VehicleManager * _vm;
    gcs::ImageFeedFilter * _filter;    
    
    QMap<int, MapWidget3D::Vehicle3D *> _vehicle_map;
    QMap<int, Qt3DCore::QEntity *> _building_map;
    
    Window3D * _view;
    
    Qt3DCore::QEntity * _root;
    Qt3DCore::QEntity * _plane;
    
    QTimer * _update_timer;
    
    QMap<int, std::shared_ptr<gcs::Building>> _buildings;
    QMap<std::shared_ptr<gcs::Point>, int> _waypoint_to_building;
};

#endif /* MAP_WIDGET_3D_H */

