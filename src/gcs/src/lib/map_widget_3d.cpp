
/* 
 * File:   map_widget_3d.cpp
 * Author: n8
 * 
 * Created on June 30, 2017, 5:11 PM
 */

#include <QVBoxLayout>
#include <QQuaternion>

//#include <Qt3DCore/QTransform>
//
//#include <Qt3DExtras/Qt3DWindow>
//#include <Qt3DExtras/QOrbitCameraController>
//#include <Qt3DExtras/QForwardRenderer>
//#include <Qt3DExtras/QCuboidGeometry>
//#include <Qt3DExtras/QCuboidMesh>
//#include <Qt3DExtras/QPlaneGeometry>
//#include <Qt3DExtras/QPlaneMesh>
//#include <Qt3DExtras/QPhongMaterial>
//
//#include <Qt3DRender/QSceneLoader>
//#include <Qt3DRender/QPointLight>
//#include <Qt3DRender/QCamera>

#include <Qt3DCore/Qt3DCore>
#include <Qt3DExtras/Qt3DExtras>
#include <Qt3DRender/Qt3DRender>

#include <gcs/qt/map_widget_3d.h>

#include <gcs/qt/vehicle_manager.h>

using namespace Qt3DCore;
using namespace Qt3DExtras;
using namespace Qt3DRender;
using namespace gcs;

typedef Qt3DCore::QTransform Transform;

MapWidget3D::MapWidget3D( QWidget * parent) :
QWidget(parent),
_view(new Qt3DWindow),
_root(new QEntity),
_plane(new QEntity(_root))
{
    setupUi();
    
    _view->defaultFrameGraph()->setClearColor(QColor(QRgb(0x0f0f0f))); //0x4d4d4f
    _view->setRootEntity(_root);
    
    createDefaultScene();
}


MapWidget3D::~MapWidget3D() 
{
}

void MapWidget3D::setVehicleManager(VehicleManager* vm)
{
    _vm = vm;
}

void MapWidget3D::setupUi() 
{   
    QWidget * container = QWidget::createWindowContainer(_view);
    
    container->setContentsMargins(0, 0, 0, 0);
    
    QVBoxLayout * layout = new QVBoxLayout();
    layout->setContentsMargins(0, 0, 0, 0);
    setLayout(layout);
    
    layout->addWidget(container, 1);
}

void MapWidget3D::createDefaultScene()
{   
    createCameraController();
    
    float intensity = 0.7;
    createLighting({30,30,30}, intensity);
    createLighting({-30,30,-30}, intensity);
    
    createFloor();
    
    float size = 4;
    createBuilding({0, size/2, 0}, size);
}

void MapWidget3D::createFloor()
{
    QPlaneMesh * plane_mesh = new QPlaneMesh();
    Transform * transform = new Transform(_plane);
    QPhongMaterial *planeMaterial = new QPhongMaterial();
    
    _plane->addComponent(plane_mesh);
    _plane->addComponent(transform);
    _plane->addComponent(planeMaterial);
    
    plane_mesh->setHeight(40);
    plane_mesh->setWidth(40);
    plane_mesh->setMeshResolution(QSize(4,4));
    
    transform->setScale(1.0f);
    transform->setTranslation(QVector3D(0.0f, 0.0f, 0.0f));
    
    planeMaterial->setDiffuse(QColor(QRgb(0xa69929)));
}

void MapWidget3D::createCameraController()
{
    Qt3DRender::QCamera *cam = _view->camera();

    cam->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    cam->setPosition(QVector3D(0, 10, 20.0f));
    cam->setUpVector(QVector3D(0, 1, 0));
    cam->setViewCenter(QVector3D(0, 0, 0));
    
    QOrbitCameraController *camController = new QOrbitCameraController(_root);
    camController->setCamera(cam);
    camController->setZoomInLimit(0.4);
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

