
/* 
 * File:   map_widget_3d.cpp
 * Author: n8
 * 
 * Created on June 30, 2017, 5:11 PM
 */

#include <QFrame>
#include <QRect>
#include <QPoint>
#include <QVBoxLayout>
#include <QQuaternion>

#include <gcs/util/debug.h>

#include <Qt3DCore/Qt3DCore>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DExtras>
#include <Qt3DRender/Qt3DRender>
#include <Qt3DRender/QViewport>

#include <gcs/qt/map_widget_3d.h>
#include <gcs/qt/ui_adapter.h>

#include <gcs/qt/vehicle_manager.h>
#include <vehicle/vehicle_control.h>

#include "gcs/qt/multi_viewport_forward_renderer.h"
#include "gcs/qt/window_3d.h"

#define MINI_MAP_STYLE "background-color: rgb(48, 57, 80);"
#define MINI_MAP_STYLE2 "background-color: yellow;"

#define M2F 3.28084 //meters to feet
#define F2M 0.3048  //feet to meters

using namespace Qt3DCore;
using namespace Qt3DExtras;
using namespace Qt3DRender;
using namespace gcs;

//static QTime sec = QTime::currentTime();

void MapWidget3D::Vehicle3D::update()
{
    // NOTE: z and y axes are swapped between ros and Qt3d
    // prompting the following translations: 
    // rotate yaw by 90 degrees
    // negate z position after swapping z and y
    Position pos = _vehicle->getPosition();
    
    QVector3D vec( pos.position.x, 
                   pos.position.z, 
                  -pos.position.y);
    //vec = vec * 1/transform->scale();
    //first set position with z and y swapped
  _transform->setTranslation(vec);
    
    //next orientation, also with z and y axes swapped
  _transform->setRotationX(pos.orientation.pitch);
  _transform->setRotationY(pos.orientation.yaw + 90);
  _transform->setRotationZ(pos.orientation.roll);
}

MapWidget3D::MapWidget3D( QWidget * parent) :
QWidget(parent),
_view(new Window3D),
_update_timer(nullptr)
{   
    setupUi();
    connectToUiAdapter();
 
    _root = _view->sceneRoot();
    _plane = new QEntity(_root);
    
    createDefaultScene();
}


MapWidget3D::~MapWidget3D() 
{
}

void MapWidget3D::setImageFeedFilter(gcs::ImageFeedFilter * filter)
{
    _view->installEventFilter(filter);
}

void MapWidget3D::setVehicleManager(VehicleManager* vm)
{
    _vm = vm;
}

void MapWidget3D::setUpdateTimer(QTimer * timer)
{
    if(_update_timer)
    {
        QObject::disconnect(_update_timer, &QTimer::timeout,
                            this, &MapWidget3D::update);
    }
    
    _update_timer = timer;
    
    QObject::connect(_update_timer, &QTimer::timeout,
                     this, &MapWidget3D::update);
}


void MapWidget3D::update()
{
    for(auto it = _vehicle_map.constBegin(); it != _vehicle_map.constEnd(); ++it)
    {
        Vehicle3D * v = *it;
        
        v->update();
        
        
        
    }
    
    
}

void MapWidget3D::vehicleAdded(int v_id)
{
    Vehicle3D * v = createVehicle(_vm->VehicleTypeFromId(v_id));
    v->_vehicle = _vm->GetVehicle(v_id);
    v->_entity->setParent(_root);
    
    _vehicle_map.insert(v_id, v);
}

void MapWidget3D::createDefaultScene()
{   
    createCameraController();
    
    float intensity = 0.5;
    float pos = 40;
    createLighting({pos,pos,pos}, intensity);
    createLighting({-pos,pos,-pos}, intensity);
    createLighting({-pos,pos,pos}, intensity);
    createLighting({pos,pos,-pos}, intensity);
    
    createFloor();
    
    float size = 1; 
    createBuilding({0, size/2, 0}, size);
}

void MapWidget3D::createFloor()
{
    QPlaneMesh * plane_mesh = new QPlaneMesh();
    Transform * transform = new Transform();
    QPhongMaterial *planeMaterial = new QPhongMaterial();
    
    _plane->addComponent(plane_mesh);
    _plane->addComponent(transform);
    _plane->addComponent(planeMaterial);
    
    //int width = 40; * F2M;
    int res = 4; // * F2M;
    
    plane_mesh->setHeight(30 * F2M);
    plane_mesh->setWidth(60 * F2M);
    plane_mesh->setMeshResolution(QSize(res,res));
    
    transform->setScale(1.0f);
    transform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    
    //QRgb(0xa69929)
    planeMaterial->setDiffuse(QColor(QRgb(0x555555)));
}

void MapWidget3D::createCameraController()
{
    Qt3DRender::QCamera *cam = _view->camera();
    
    cam->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cam->setPosition(QVector3D(-20, 10, 0)); //looking forward along x axis to origin
    cam->setUpVector(QVector3D(0, 1, 0));
    cam->setViewCenter(QVector3D(0, 0, 0));
    
    QOrbitCameraController *camController = new QOrbitCameraController(_root);
    camController->setCamera(cam);
    camController->setZoomInLimit(5);
    camController->setLinearSpeed(60);
    camController->setLookSpeed(60);
    
    /////////////////
    
    cam = _view->miniMapCamera();
    cam->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cam->setPosition(QVector3D(-1, 20, 0));
    cam->setUpVector(QVector3D(0, 1, 0));
    cam->setViewCenter(QVector3D(0, 0, 0));
}

void MapWidget3D::createLighting(const QVector3D& pos, float intensity)
{
    QEntity *entity = new QEntity(_root);
    QPointLight *light = new QPointLight(entity);
    light->setColor("white");
    light->setIntensity(intensity);
    entity->addComponent(light);
    Transform *transform = new Transform();
    transform->setTranslation(pos);
    entity->addComponent(transform);
}

void MapWidget3D::createBuilding(const QVector3D& pos, float size)
{
    QEntity *entity = new QEntity(_root);
    
    // Cuboid shape data
    QCuboidMesh *mesh = new QCuboidMesh();
    
    // CuboidMesh Transform
    Transform *transform = new Transform();
    transform->setScale(size);
    transform->setTranslation(pos);
    
    QPhongMaterial *cuboidMaterial = new QPhongMaterial();
    cuboidMaterial->setDiffuse(QColor(QRgb(0x665423)));

    entity->addComponent(mesh);
    entity->addComponent(cuboidMaterial);
    entity->addComponent(transform);
}

MapWidget3D::Vehicle3D * MapWidget3D::createVehicle(int vehicle_type)
{
    //todo dont ignore vehicle_type
    if(vehicle_type == VehicleType::quad_rotor)
    {
        QEntity * entity = new QEntity();        
        
        QMesh * mesh = new QMesh();
        mesh->setSource(QUrl("qrc:/vehicles/QuadRotor.obj"));
        
        Transform * transform = new Transform();
        transform->setScale(0.002);
        transform->setTranslation({6, 4, 6});
        transform->setRotationY(220);
        
        QPhongMaterial * material = new QPhongMaterial();
        material->setDiffuse(QColor(QRgb(0x928327)));
        
        entity->addComponent(mesh);
        entity->addComponent(transform);
        entity->addComponent(material);
        
        Vehicle3D * v3d = new Vehicle3D;
        
        v3d->_entity = entity;
        v3d->_mesh = mesh;
        v3d->_material = material;
        v3d->_transform = transform;
        
        return v3d;
    }
    
    return nullptr;
}

void MapWidget3D::connectToUiAdapter()
{
    UIAdapter * uia = UIAdapter::Instance();
    QObject::connect(uia, &UIAdapter::vehicleAdded,
                    this, &MapWidget3D::vehicleAdded);
}

    
void MapWidget3D::setupUi() 
{   
    QVBoxLayout * layout = new QVBoxLayout(this);
    QWidget * container = QWidget::createWindowContainer(_view);
        
    container->setContentsMargins(0, 0, 0, 0);
    layout->setContentsMargins(0, 0, 0, 0);

    setLayout(layout);
    layout->addWidget(container);
}

