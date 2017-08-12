
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

#include <Qt3DCore>
#include <Qt3DExtras>
#include <Qt3DRender>

#include <gcs/qt/map_widget_3d.h>
#include <gcs/qt/multi_viewport_forward_renderer.h>
#include <gcs/qt/window_3d.h>
#include <gcs/qt/trial_manager.h>
#include <gcs/qt/ui_adapter.h>
#include <gcs/qt/vehicle_manager.h>
#include <gcs/qt/image_feed_filter.h>

#include <gcs/util/debug.h>

#include <vehicle/vehicle_control.h> 


using namespace Qt3DCore;
using namespace Qt3DExtras;
using namespace Qt3DRender;
using namespace gcs;


#define M2F 3.28084 // meters to feet
#define F2M 0.3048  // feet to meters
#define B_SIZE ((float) (3.0 * F2M)) // building size
#define F_SIZE ((float) (16.0 * F2M)) // floor size
#define V_SIZE 0.0013 // uav size

 // NOTE: z and y axes are swapped between ros and Qt3d
    // prompting the following translations: 
    // rotate yaw by 90 degrees
    // negate z position after swapping z and y

//#define transformedVec(x, y, z) QVector3D(x, z, -y)
#define transformedVec(x, y, z) QVector3D(x, z  , -y)

#define rotateY(obj, y) obj->setRotationY(y + 90)

void MapWidget3D::Vehicle3D::update()
{
    Position pos = _vehicle->getPosition();
    
    int z =  pos.position.z > 0 ?
        pos.position.z :
        0;
    
    QVector3D vec = transformedVec(pos.position.x,
                                   pos.position.y, 
                                   z);
    
    //vec.setX(vec.x() + 0.5);
    //first set position with z and y swapped
  _transform->setTranslation(vec);
    
    //next orientation, also with z and y axes swapped
  _transform->setRotationX(pos.orientation.pitch);
  rotateY(_transform, pos.orientation.yaw);
  _transform->setRotationZ(pos.orientation.roll);
}


///////////////////////


MapWidget3D::MapWidget3D( QWidget * parent) :
QWidget(parent),
_view(new Window3D),
_update_timer(nullptr)
{   
    _renderer = _view->renderer();
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
    _image_filter = filter;
    _view->installEventFilter(filter);
}

void MapWidget3D::setTrialManager(gcs::TrialManager * trial_manager)
{
    _trial_manager = trial_manager;
    
    if(_trial_manager)
    {
        QObject::connect(_trial_manager, &TrialManager::trialChanged,
                         this, &MapWidget3D::trialChanged);

        QObject::connect(_trial_manager, &TrialManager::sigReset,
                         this, &MapWidget3D::reset);
        
        QObject::connect(_trial_manager, &TrialManager::currentBuildingChanged,
                         this, [this]() 
        {
            checkBuildingState(_cur_building);
            _cur_building = _trial_manager->currentBuilding();
        });
                         
        QObject::connect(_trial_manager, &TrialManager::accessPointFound,
                         this, [this](BuildingID building_id, FoundBy found_by)
        {
            auto buildings = _trial_manager->getBuildings();
            std::shared_ptr<Building3D> b3d = _buildings_3d[building_id];
            
            if(b3d == nullptr)
                return;
            
            if(found_by != Building::fNull)
                b3d->_material->setDiffuse(QColor("blue"));
        });
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
        std::shared_ptr<Vehicle3D> v = *it;
        v->update();
    }
    
    
    if(!_cur_vehicle)
        return;

    //auto b = _waypoint_to_building[_cur_vehicle->currentWaypoint()];
//    _image_filter->setCurrentBuilding(b);
    
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
    std::shared_ptr<Vehicle3D> v = createVehicle(_vm->VehicleTypeFromId(v_id));
    v->_vehicle = _vm->GetVehicle(v_id);
    v->_entity->setParent(_root);
    
    _vehicle_map.insert(v_id, v);
}

void MapWidget3D::trialChanged()
{
    //_waypoint_to_building.clear();
    
    auto buildings = _trial_manager->getBuildings();
    auto waypoints = _trial_manager->getWaypointInfoList();
    
    for(int i = 0; i <  waypoints.size(); i++)
    {
        auto wp = waypoints[i];
        if(wp->building_id >= buildings.size())
        {
            qCDebug(lcar_bot) << "MORE WAYPOINT BUILDING ID's THAN BLUILDINGS";
            continue;
        }
        
        //_waypoint_to_building.insert(i, buildings[wp->building_id]);
    }
    
    loadScene();
}

void MapWidget3D::reset()
{ 
    for(const auto& v : _vehicle_map)
    {
        v->_entity->setParent((QEntity*)(nullptr));
    }
    
    _root = new QEntity();
    _view->setSceneRoot(_root);
    
    for(const auto& v : _vehicle_map)
    {
        v->_entity->setParent(_root);
    }  
    
    createCameraController();
    
    float intensity = 0.5;
    float pos = 20;
    createLighting({pos,pos,pos}, intensity);
    createLighting({-pos,pos,-pos}, intensity);
    createLighting({-pos,pos,pos}, intensity);
    createLighting({pos,pos,-pos}, intensity);
    
    createFloor(); 
}

void MapWidget3D::loadBuildings()
{    
    if(!_trial_manager)
        return;
    
    qCDebug(lcar_bot) << "map_widget_3d loading buildings:" << _trial_manager->getBuildings().size();
    
    for(const auto& building : _trial_manager->getBuildings())
    {
        createBuilding(building);
    }
}

void MapWidget3D::loadScene()
{   
    reset();
    loadBuildings();
    _view->fixAspectRatio();
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
     _plane = new QEntity(_root);
    
    QPlaneMesh * plane_mesh = new QPlaneMesh();
    Transform * transform = new Transform();
    QPhongMaterial *planeMaterial = new QPhongMaterial();
    
    _plane->addComponent(plane_mesh);
    _plane->addComponent(transform);
    _plane->addComponent(planeMaterial);
    
    //int width = 40; * F2M;
    int res = 4; // * F2M;
    
    plane_mesh->setHeight(F_SIZE);
    plane_mesh->setWidth(F_SIZE);
    
    plane_mesh->setMeshResolution(QSize(res,res));
    
    transform->setScale(1);
    transform->setTranslation(transformedVec(0, 0, 0));
    rotateY(transform, 0);
    
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
    cam->setPosition(QVector3D(-1, 10, 0));
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

void MapWidget3D::createBuilding(const QVector3D& pos, float size, QColor color)
{
    QEntity *entity = new QEntity(_root);
    // Cuboid shape data
    QCuboidMesh *mesh = new QCuboidMesh();

    // CuboidMesh Transform
    Transform *transform = new Transform();
    transform->setScale(size);
    transform->setTranslation(pos);
    rotateY(transform, 0);
    
    
    QPhongMaterial *cuboidMaterial = new QPhongMaterial();
    cuboidMaterial->setDiffuse(color);

    entity->addComponent(mesh);
    entity->addComponent(cuboidMaterial);
    entity->addComponent(transform);    
}

void MapWidget3D::createBuilding(const std::shared_ptr<Building>& b)
{   
//    if(_layer == nullptr)
//        _layer = new QLayer(_root);
//    
//    if(_layer_filter == nullptr)
//    {
//        _layer_filter = new QLayerFilter(_renderer->m_viewport);
//        _layer_filter->addLayer(_layer);
//        
//        _renderer->m_cameraSelector->setParent((QNode*)nullptr);
//        _renderer->m_cameraSelector->setParent(_layer_filter);
//        
//        _renderer->m_clearBuffer->setParent((QNode*)nullptr);
//        _renderer->m_clearBuffer->setParent(_renderer->m_cameraSelector);
//    }
    
    QColor c;
    switch(b->buildingType())
    {
        case Building::tPurple:
            c = QColor("purple");
            break;
        case Building::tWhite:
            c = QColor("white");
            break;
        case Building::tNull:
        default:
            c = QColor("orange");
            break;
    }
    
    QVector3D pos = transformedVec(b->xPos(), b->yPos(), B_SIZE/2);
    
    //////////////////
    
    std::shared_ptr<Building3D> b3 = std::make_shared<Building3D>();
    b3->id = b->getID();
    
    b3->_entity_large = new QEntity(_root);

    // CuboidMesh Transform
    b3->_transform_large = new Transform();
    b3->_transform_large->setScale(B_SIZE);
    b3->_transform_large->setTranslation(pos);
    rotateY(b3->_transform_large, 0);
    
    
    b3->_material = new QPhongMaterial();
    b3->_material->setDiffuse(c);

    b3->_entity_large->addComponent(new QCuboidMesh());
    b3->_entity_large->addComponent(b3->_material);
    b3->_entity_large->addComponent(b3->_transform_large);
    //b3->_entity_large->addComponent(_layer);

    //b3->_layer = new QLayer(b3->_entity_large);
    //b3->_filter = new QLayerFilter(_renderer->m_viewport);

    ///////////////////
//    b3->_entity_mini = new QEntity(_root);
//
//    b3->_transform_mini = new Transform();
//    b3->_transform_mini->setScale(B_SIZE);
//    b3->_transform_mini->setTranslation(pos);
//
//    c = QColor("red");
//
//    b3->_cube_mat_mini = new QPhongMaterial();
//    b3->_cube_mat_mini->setDiffuse(c);
//
//    b3->_entity_mini->addComponent(new QCuboidMesh());
//    b3->_entity_mini->addComponent(b3->_cube_mat_mini);
//    b3->_entity_mini->addComponent(b3->_transform_mini);
        
    ////////////
    
    _buildings_3d.insert(b3->id, b3);
}

std::shared_ptr<MapWidget3D::Vehicle3D> MapWidget3D::createVehicle(int vehicle_type)
{
    //todo dont ignore vehicle_type
    if(vehicle_type == VehicleType::quad_rotor)
    {
        QEntity * entity = new QEntity();        
        
        QMesh * mesh = new QMesh();
        mesh->setSource(QUrl("qrc:/vehicles/QuadRotor.obj"));
        
        Transform * transform = new Transform();
        transform->setScale(V_SIZE);
        
        QPhongMaterial * material = new QPhongMaterial();
        material->setDiffuse(QColor(QRgb(0x928327)));
        
        entity->addComponent(mesh);
        entity->addComponent(transform);
        entity->addComponent(material);
        
        std::shared_ptr<Vehicle3D> v3d = std::make_shared<Vehicle3D>();
        
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
    
    QObject::connect(uia, &UIAdapter::DeleteVehicle,
                    this, [this](int id)
    {
        if(_cur_vehicle->id == id)
            _cur_vehicle = nullptr;
        
        auto v_3d = _vehicle_map.take(id);
        if(v_3d)
        {
            v_3d->_entity->deleteLater();
        }
        
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

void MapWidget3D::setBuildingColor(std::shared_ptr<MapWidget3D::Building3D> b3d, const QColor& color)
{
    if(b3d == nullptr)
        return;

    b3d->_material->setDiffuse(color);
}

void MapWidget3D::checkBuildingState(const std::shared_ptr<gcs::Building>& building)
{
    if(!building)
        return;

    auto b3d = _buildings_3d[building->getID()];
    
    Q_ASSERT(b3d);
    if(b3d == nullptr)
        return;

    //ignore color change if the building was already found by vehicle
    FoundBy f = building->foundBy();
    if(f != Building::fNull)
        return;

    if(building->foundByTentative() == Building::fNull)
        setBuildingColor(b3d, QColor("red"));
    else
        setBuildingColor(b3d, QColor("blue"));
}
