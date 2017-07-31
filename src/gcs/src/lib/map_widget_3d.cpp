
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

#include <Qt3DCore/Qt3DCore>
#include <Qt3DCore/QTransform>
#include <Qt3DExtras/Qt3DExtras>
#include <Qt3DRender/Qt3DRender>
#include <Qt3DRender/QViewport>

#include <gcs/qt/map_widget_3d.h>
#include <gcs/qt/multi_viewport_forward_renderer.h>
#include <gcs/qt/window_3d.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/vehicle_manager.h>
#include <gcs/qt/image_feed_filter.h>

#include <gcs/util/debug.h>

#include <vehicle/vehicle_control.h> 

#define M2F 3.28084 // meters to feet
#define F2M 0.3048  // feet to meters

#define SIZE 0.5
#define SCALE 2;

#define VEHICLE_SCALE 0.001

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
    
    int z =  pos.position.z > 0 ?
        pos.position.z :
        0;
    
    QVector3D vec( pos.position.x, 
                   z, 
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
    loadScene();
}


MapWidget3D::~MapWidget3D() 
{
}

 void MapWidget3D::setCurrentVehicle(gcs::VehicleControl * vehicle)
 {
     _cur_vehicle = vehicle;
 }

void MapWidget3D::setImageFeedFilter(gcs::ImageFeedFilter * filter)
{
    _filter = filter;
    _view->installEventFilter(filter);
}

void MapWidget3D::setTrialManager(gcs::TrialManager * trial_manager)
{
    if(_trial_manager)
    {
        QObject::disconnect(_trial_manager, &TrialManager::trialChanged,
                            this, &MapWidget3D::trialChanged);
        
        QObject::disconnect(_trial_manager, &TrialManager::sigReset,
                            this, &MapWidget3D::reset);
    }

    _trial_manager = trial_manager;
    
    if(_trial_manager)
    {
        QObject::connect(_trial_manager, &TrialManager::trialChanged,
                         this, &MapWidget3D::trialChanged);

        QObject::connect(_trial_manager, &TrialManager::sigReset,
                         this, &MapWidget3D::reset);
    }
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
                            this, &MapWidget3D::positionUpdate);
    }
    
    _update_timer = timer;
    
    QObject::connect(_update_timer, &QTimer::timeout,
                     this, &MapWidget3D::positionUpdate);
}


void MapWidget3D::positionUpdate()
{
    for(auto it = _vehicle_map.constBegin(); it != _vehicle_map.constEnd(); ++it)
    {
        Vehicle3D * v = *it;
        v->update();
    }
    
    
    if(!_cur_vehicle)
        return;

    auto b = _waypoint_to_building[_cur_vehicle->currentWaypoint()];
    _filter->setCurrentBuilding(b);
    
//    int v_id = _cur_vehicle->id;
//    
//    QVector3D vec = _vehicle_map.value(v_id)->_transform->translation();
//    
//    vec.setY(vec.y() + 4);
//  
//    _view->camera()->setPosition(vec);
//    
//    ////////////
//    
//    vec.setY(vec.y() - 4);
//    camera->setViewCenter(vec);
}

void MapWidget3D::vehicleAdded(int v_id)
{
    Vehicle3D * v = createVehicle(_vm->VehicleTypeFromId(v_id));
    v->_vehicle = _vm->GetVehicle(v_id);
    v->_entity->setParent(_root);
    
    _vehicle_map.insert(v_id, v);
}

void MapWidget3D::trialChanged()
{
    _waypoint_to_building.clear();
    
    auto buildings = _trial_manager->getBuildings();
    auto waypoints = _trial_manager->getWaypointInfoList();
    
    for(int i = 0; i <  waypoints.length(); i++)
    {
        auto wp = waypoints[i];
        _waypoint_to_building.insert(i, buildings[wp->building_id]);
    }
    
    loadScene();
}

void MapWidget3D::reset()
{ 
    for(const auto& v : _vehicle_map)
    {
        v->_entity->setParent(static_cast<QEntity*>(nullptr));
    }
    _root = new QEntity();
    _view->setSceneRoot(_root);
    
    createCameraController();
    
    float intensity = 0.5;
    float pos = 20;
    createLighting({pos,pos,pos}, intensity);
    createLighting({-pos,pos,-pos}, intensity);
    createLighting({-pos,pos,pos}, intensity);
    createLighting({pos,pos,-pos}, intensity);
    
     _plane = new QEntity(_root);
    createFloor();
    
    for(const auto& v : _vehicle_map)
    {
        v->_entity->setParent(_root);
    }
}

void MapWidget3D::loadBuildings()
{    
    if(!_trial_manager)
        return;
    
    qCDebug(lcar_bot) << "MAP 3D B:" << _trial_manager->getBuildings().size();
    
    for(const auto& building : _trial_manager->getBuildings())
    {
        QColor c;
        switch(building->buildingType())
        {
            case Building::tPurple:
                c = QColor("purple");
                break;
            case Building::tWhite:
                c = QColor("white");
                break;
            case Building::tNull:
            default:
                c = QColor("red");
                break;
        }
        
        QVector3D vec(building->xPos(), (float)SIZE/2, building->yPos());
        
        createBuilding(vec, SIZE, c);
    }
}

void MapWidget3D::loadScene()
{   
    reset();
    loadBuildings();
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
    createBuilding({0, size/2, 0}, size, "purple");
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
    
    plane_mesh->setHeight(16 
                          * F2M
                          );
    plane_mesh->setWidth(16 
                         * F2M
                         );
    
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
    cam->setPosition(QVector3D(-16 * F2M, 10, 0)); //looking forward along x axis to origin
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
    cam->setPosition(QVector3D(-0.4, 10, 2.5));
    cam->setUpVector(QVector3D(0, 1, 0));
    cam->setViewCenter(QVector3D(0, 0, 2.5));
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

void MapWidget3D::createBuilding(const QVector3D& pos, float size, QColor color)
{
    QEntity *entity = new QEntity(_root);
    // Cuboid shape data
    QCuboidMesh *mesh = new QCuboidMesh();
    
    // CuboidMesh Transform
    Transform *transform = new Transform();
    transform->setScale(size);
    transform->setTranslation(pos);
    
    QPhongMaterial *cuboidMaterial = new QPhongMaterial();
    cuboidMaterial->setDiffuse(color);

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
        transform->setScale(VEHICLE_SCALE);
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
    
    QObject::connect(uia, &UIAdapter::setCurrentVehicle,
                    this, [this](int id)
    {
        _cur_vehicle = _vm->GetVehicle(id);
    });
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
