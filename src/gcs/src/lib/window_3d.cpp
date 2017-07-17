/****************************************************************************
**
** Copyright (C) 2016 Klaralvdalens Datakonsult AB (KDAB).
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt3D module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** BSD License Usage
** Alternatively, you may use this file under the terms of the BSD license
** as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of The Qt Company Ltd nor the names of its
**     contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/


/* 
 * File:   Window3D.cpp
 * Author: n8
 * 
 * Created on July 16, 2017, 6:55 PM
 */

#include <gcs/qt/window_3d.h>
#include <gcs/qt/multi_viewport_forward_renderer.h>

#include <Qt3DExtras/qforwardrenderer.h>

#include <Qt3DRender>
#include <Qt3DExtras>

using namespace Qt3DCore;
using namespace Qt3DRender;
using namespace Qt3DExtras;

Window3D::Window3D(QScreen *screen)
    : QWindow(screen)
    , m_aspectEngine(new Qt3DCore::QAspectEngine)
    , m_renderAspect(new Qt3DRender::QRenderAspect)
    , m_inputAspect(new Qt3DInput::QInputAspect)
    , m_logicAspect(new Qt3DLogic::QLogicAspect)
    , m_renderSettings(new Qt3DRender::QRenderSettings)
    , m_forwardRenderer(new MultiViewportForwardRenderer)
    //, m_forwardRenderer(new Qt3DExtras::QForwardRenderer)
    , m_defaultCamera(new Qt3DRender::QCamera)
    , _mini_camera(new Qt3DRender::QCamera)
    , m_inputSettings(new Qt3DInput::QInputSettings)
    , _root(new Qt3DCore::QEntity)
    , _scene_root(new Qt3DCore::QEntity)
    , _init(false)
{
    setSurfaceType(QSurface::OpenGLSurface);

    QSurfaceFormat format;
#ifdef QT_OPENGL_ES_2
    format.setRenderableType(QSurfaceFormat::OpenGLES);
#else
    if (QOpenGLContext::openGLModuleType() == QOpenGLContext::LibGL) {
        format.setVersion(4, 3);
        format.setProfile(QSurfaceFormat::CoreProfile);
    }
#endif
    format.setDepthBufferSize(24);
    format.setSamples(4);
    format.setStencilBufferSize(8);
    setFormat(format);
    QSurfaceFormat::setDefaultFormat(format);

    m_aspectEngine->registerAspect(m_renderAspect);
    m_aspectEngine->registerAspect(m_inputAspect);
    m_aspectEngine->registerAspect(m_logicAspect);

    m_defaultCamera->setParent(_root);
    _mini_camera->setParent(_root);
    m_forwardRenderer->setCamera(m_defaultCamera);
    m_forwardRenderer->setMiniMapCamera(_mini_camera);
    m_forwardRenderer->setSurface(this);
    m_renderSettings->setActiveFrameGraph(m_forwardRenderer);
    m_inputSettings->setEventSource(this);
    
    _scene_root->setParent(_root);
}

Window3D::~Window3D() {
}


void Window3D::showEvent(QShowEvent *e)
{
    if (!_init) 
    {
        _root->addComponent(m_renderSettings);
        _root->addComponent(m_inputSettings);
        m_aspectEngine->setRootEntity(Qt3DCore::QEntityPtr(_root));
        
        m_forwardRenderer->init();
        
        _init = true;
    }

    QWindow::showEvent(e);
}

void Window3D::resizeEvent(QResizeEvent *)
{
    m_defaultCamera->setAspectRatio(float(width()) / float(height()));
    _mini_camera->setAspectRatio(float(width()) / float(height()));
}
