
/* 
 * File:   multi_viewport_forward_renderer.h
 * Author: n8
 *
 * Created on July 16, 2017, 5:32 PM
 */

#ifndef MULTI_VIEWPORT_FORWARD_RENDERER_H
#define MULTI_VIEWPORT_FORWARD_RENDERER_H

#include <Qt3DRender/QTechniqueFilter>

namespace Qt3DExtras
{
    class QViewport;
    class QForwardRendererPrivate;
}

namespace Qt3DRender
{
    class QCamera;
    class QRenderSurfaceSelector; //*m_surfaceSelector;
    class QViewport; // *m_viewport;
    class QCameraSelector; // *m_cameraSelector;
    class QClearBuffers; // *m_clearBuffer;
    class QFrustumCulling; // *m_frustumCulling;
}

class MultiViewportForwardRenderer : public Qt3DRender::QTechniqueFilter
{
    Q_OBJECT
public:
    MultiViewportForwardRenderer(Qt3DCore::QNode *parent = nullptr);
    virtual ~MultiViewportForwardRenderer();
    
    void init();
    
    void setCamera(Qt3DRender::QCamera * camera);
    void setMiniMapCamera(Qt3DRender::QCamera * camera);
    
    void setSurface(QObject *surface);
    
private:
    Qt3DRender::QRenderSurfaceSelector *m_surfaceSelector;
    //Qt3DRender::QRenderSurfaceSelector *m_surfaceSelector_mini;
    
    
    Qt3DRender::QViewport *m_viewport_container;
    Qt3DRender::QViewport *m_viewport;
    Qt3DRender::QViewport *m_viewport_mini;
    
    Qt3DRender::QCameraSelector *m_cameraSelector;
    Qt3DRender::QCameraSelector *m_cameraSelector_mini;
    
    Qt3DRender::QClearBuffers *m_clearBuffer;
    //Qt3DRender::QClearBuffers *m_clearBuffer_mini;
    
    Qt3DRender::QFrustumCulling *m_frustumCulling;  
    //Qt3DRender::QFrustumCulling *m_frustumCulling_mini;  
};

#endif /* MULTI_VIEWPORT_FORWARD_RENDERER_H */

