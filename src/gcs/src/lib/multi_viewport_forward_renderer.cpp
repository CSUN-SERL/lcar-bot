
/* 
 * File:   multi_viewport_forward_renderer.cpp
 * Author: n8
 * 
 * Created on July 16, 2017, 5:32 PM
 */

#include <Qt3DCore/qentity.h>
#include <Qt3DRender/qviewport.h>
#include <Qt3DRender/qcameraselector.h>
#include <Qt3DRender/qclearbuffers.h>
#include <Qt3DRender/qfilterkey.h>
#include <Qt3DRender/qfrustumculling.h>
#include <Qt3DRender/qrendersurfaceselector.h>
#include <Qt3DRender/QCamera>

#include <gcs/qt/multi_viewport_forward_renderer.h>

using namespace Qt3DRender;

MultiViewportForwardRenderer::MultiViewportForwardRenderer(Qt3DCore::QNode *parent) : 
QTechniqueFilter(parent)
    , m_surfaceSelector(new QRenderSurfaceSelector)
    //, m_surfaceSelector_mini(new QRenderSurfaceSelector)
    , m_viewport_container(new QViewport())
    , m_viewport(new QViewport())
    , m_viewport_mini(new QViewport())
    , m_cameraSelector(new QCameraSelector())
    , m_cameraSelector_mini(new QCameraSelector())
    , m_clearBuffer(new QClearBuffers())
    , m_clearBuffer_mini(new QClearBuffers())
    , m_frustumCulling(new QFrustumCulling())
    , m_frustumCulling_mini(new QFrustumCulling())
{
}

MultiViewportForwardRenderer::~MultiViewportForwardRenderer() 
{
}

void MultiViewportForwardRenderer::init()
{    
    m_surfaceSelector->setParent(this);
    m_viewport_container->setParent(m_surfaceSelector);
    
    m_viewport->setParent(m_viewport_container);
    m_cameraSelector->setParent(m_viewport);
    
    m_viewport_mini->setParent(m_viewport_container);
    m_cameraSelector_mini->setParent(m_viewport_mini);
    
    m_clearBuffer->setClearColor(QColor(QRgb(0x0f0f0f)));
    m_clearBuffer->setBuffers(QClearBuffers::ColorDepthBuffer);
    
    //m_viewport_container->setNormalizedRect(QRectF(0.0f, 0.0f, 1.0f, 1.0f));
    m_viewport->setNormalizedRect(QRectF(0.0f, 0.0f, 1.0f, 1.0f));
    m_viewport_mini->setNormalizedRect(QRectF(0.0f, 0.75f, 0.25f, 0.25f));

    m_clearBuffer->setParent(m_cameraSelector);
    m_frustumCulling->setParent(m_clearBuffer);
    
    ////
    
    //m_clearBuffer_mini->setParent(m_cameraSelector_mini);
    //m_frustumCulling_mini->setParent(m_clearBuffer_mini);
//    
    //m_clearBuffer_mini->setClearColor(QColor(QRgb(0x0f0f0f)));
    //m_clearBuffer_mini->setBuffers(QClearBuffers::ColorDepthBuffer);
    
    QFilterKey *forwardRenderingStyle = new QFilterKey(this);
    forwardRenderingStyle->setName(QStringLiteral("renderingStyle"));
    forwardRenderingStyle->setValue(QStringLiteral("forward"));
    addMatch(forwardRenderingStyle);
}

void MultiViewportForwardRenderer::setCamera(Qt3DRender::QCamera * camera)
{
    m_cameraSelector->setCamera(camera);
}
void MultiViewportForwardRenderer::setMiniMapCamera(Qt3DRender::QCamera * camera)
{
    m_cameraSelector_mini->setCamera(camera);
}

void MultiViewportForwardRenderer::setSurface(QObject *surface)
{
    m_surfaceSelector->setSurface(surface);
    //m_surfaceSelector_mini->setSurface(surface);
}
