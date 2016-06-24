/*
# Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef __NVXIO_OPENGL_RENDER_IMPL_HPP__
#define __NVXIO_OPENGL_RENDER_IMPL_HPP__

#include <vector>
#include <memory>

#ifdef USE_GUI
# include <NVXIO/OpenGL.hpp>
#endif

#include <NVXIO/Render.hpp>
#include "Render/CUDA-OpenGL/OpenGLBasicRenders.hpp"

namespace nvxio {

class OpenGLRenderImpl : public nvxio::Render
{
public:
    virtual void putImage(vx_image image);
    virtual void putFeatures(vx_array location, const FeatureStyle& style);
    virtual void putLines(vx_array lines, const LineStyle& style);
    virtual void putMotionField(vx_image field, const MotionFieldStyle& style);
    virtual void putCircles(vx_array circles, const CircleStyle& style);
    virtual void putObjectLocation(const vx_rectangle_t& location, const DetectedObjectStyle& style);
    virtual void putConvexPoligon(vx_array verticies, const LineStyle& style);
    virtual void putText(const std::string& text, const TextBoxStyle& style);
    virtual void putArrows(vx_array old_points, vx_array new_points, const LineStyle& line_style);

    virtual vx_uint32 getWidth() const;
    virtual vx_uint32 getHeight() const;

protected:
    OpenGLRenderImpl(TargetType type, const std::string& name);
    virtual ~OpenGLRenderImpl();

    bool initGL(vx_context context,
                vx_uint32 wndWidth, vx_uint32 wndHeight,
                bool visible);

    void finalGL();

    virtual void setOpenGlContext() = 0;

    void clearGlBuffer();

    std::shared_ptr<GLFunctions> gl_;
    vx_uint32 wndWidth_;
    vx_uint32 wndHeight_;

    // color texture for custom framebuffer
    GLuint fboTex_;

private:
    vx_float32 scaleRatio_;

    ImageRender imageRender_;
    FeaturesRender featuresRender_;
    LinesRender linesRender_;
    MotionFieldRender motionFieldRender_;
    CirclesRender circlesRender_;
    TextRender textRender_;
    RectangleRender rectangleRender_;
    ArrowsRender arrowsRender_;

    vx_array tmpLines_;

    // custom framebuffer
    GLuint fbo_, fboOld_;
};

}

#endif
