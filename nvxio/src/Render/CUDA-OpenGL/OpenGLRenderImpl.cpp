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

#ifdef USE_GUI

#include <cmath>
#include <limits>
#include <algorithm>

#ifdef _WIN32
// cuda_gl_interop.h includes GL/gl.h, which requires Windows.h to work.
#define NOMINMAX
#include <windows.h>
#endif

#include <VX/vx.h>

#include <NVXIO/Utility.hpp>
#include <NVXIO/Application.hpp>

#include "Render/CUDA-OpenGL/OpenGLRenderImpl.hpp"

#include <cuda_gl_interop.h>

const int fontSize = 20;
//============================================================
// OpenGLRenderImpl
//============================================================

nvxio::OpenGLRenderImpl::OpenGLRenderImpl(TargetType type, const std::string& name) :
    Render(type, name),
    gl_(), wndWidth_(), wndHeight_(),
    fboTex_(), scaleRatio_(),
    imageRender_(), featuresRender_(),
    linesRender_(), motionFieldRender_(),
    circlesRender_(), textRender_(),
    tmpLines_(NULL), fbo_(), fboOld_()
{
}

nvxio::OpenGLRenderImpl::~OpenGLRenderImpl()
{
    if (tmpLines_)
    {
        vxReleaseArray(&tmpLines_);
        tmpLines_ = NULL;
    }

    if (gl_)
    {
        // bind old FBO
        if (fboOld_ != 0)
        {
            gl_->BindFramebuffer(GL_FRAMEBUFFER, fboOld_);
            NVXIO_CHECK_GL_ERROR();
        }

        if (fbo_ != 0)
        {
            gl_->DeleteFramebuffers(1, &fbo_);
            NVXIO_CHECK_GL_ERROR();
            fbo_ = 0;
        }

        if (fboTex_ != 0)
        {
            gl_->DeleteTextures(1, &fboTex_);
            NVXIO_CHECK_GL_ERROR();
            fboTex_ = 0;
        }
    }
}

vx_uint32 nvxio::OpenGLRenderImpl::getWidth() const
{
    return wndWidth_;
}

vx_uint32 nvxio::OpenGLRenderImpl::getHeight() const
{
    return wndHeight_;
}

void nvxio::OpenGLRenderImpl::putImage(vx_image image)
{
    vx_uint32 imageWidth = 0, imageHeight = 0;

    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, &imageWidth, sizeof(imageWidth)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, &imageHeight, sizeof(imageHeight)) );

    // calcuate actual ScaleRatio that will be applied to other primitives like lines, circles, etc.
    float widthRatio = static_cast<float>(wndWidth_) / imageWidth;
    float heightRatio = static_cast<float>(wndHeight_) / imageHeight;
    scaleRatio_ = std::min(widthRatio, heightRatio);

    setOpenGlContext();
    imageRender_.render(image);
}

void nvxio::OpenGLRenderImpl::putFeatures(vx_array location, const FeatureStyle& style)
{
    setOpenGlContext();
    featuresRender_.render(location, style, wndWidth_, wndHeight_, scaleRatio_);
}

void nvxio::OpenGLRenderImpl::putLines(vx_array lines, const LineStyle& style)
{
    setOpenGlContext();
    linesRender_.render(lines, style, wndWidth_, wndHeight_, scaleRatio_);
}

void nvxio::OpenGLRenderImpl::putMotionField(vx_image field, const MotionFieldStyle& style)
{
    setOpenGlContext();
    motionFieldRender_.render(field, style, wndWidth_, wndHeight_, scaleRatio_);
}

void nvxio::OpenGLRenderImpl::putCircles(vx_array circles, const CircleStyle& style)
{
    setOpenGlContext();
    circlesRender_.render(circles, style, wndWidth_, wndHeight_, scaleRatio_);
}

void nvxio::OpenGLRenderImpl::putText(const std::string& text, const TextBoxStyle& style)
{
    setOpenGlContext();
    textRender_.render(text, style, wndWidth_, wndHeight_);
}

void nvxio::OpenGLRenderImpl::putObjectLocation(const vx_rectangle_t& location, const DetectedObjectStyle& style)
{
    setOpenGlContext();

    // render border
    nvx_point4f_t lines[4];

    vx_float32 thickness2 = style.thickness / 2.0f;
    vx_float32 thickness2_ = style.thickness - thickness2;

    lines[0].x = location.start_x - thickness2;
    lines[0].y = static_cast<vx_float32>(location.start_y);
    lines[0].z = location.end_x + thickness2_;
    lines[0].w = static_cast<vx_float32>(location.start_y);

    lines[1].x = location.start_x - thickness2;
    lines[1].y = static_cast<vx_float32>(location.end_y);
    lines[1].z = location.end_x + thickness2_;
    lines[1].w = static_cast<vx_float32>(location.end_y);

    lines[2].x = static_cast<vx_float32>(location.end_x);
    lines[2].y = location.start_y + thickness2_;
    lines[2].z = static_cast<vx_float32>(location.end_x);
    lines[2].w = location.end_y - thickness2;

    lines[3].x = static_cast<vx_float32>(location.start_x);
    lines[3].y = location.start_y + thickness2_;
    lines[3].z = static_cast<vx_float32>(location.start_x);
    lines[3].w = location.end_y - thickness2;

    NVXIO_SAFE_CALL( vxTruncateArray(tmpLines_, 0) );
    NVXIO_SAFE_CALL( vxAddArrayItems(tmpLines_, 4, &lines[0], sizeof(nvx_point4f_t)) );

    LineStyle lineStyle = {
        {style.color[0], style.color[1], style.color[2], style.color[3]},
        style.thickness
    };
    linesRender_.render(tmpLines_, lineStyle, wndWidth_, wndHeight_, scaleRatio_);

    // render inner region if any
    if (style.isHalfTransparent)
    {
        rectangleRender_.render(location, style, wndWidth_, wndHeight_, scaleRatio_);
    }

    // render text if any
    if (!style.label.empty())
    {
        vx_coordinates2d_t textOrigin = {
            location.start_x,
            location.start_y - fontSize - 5
        };
        TextBoxStyle textStyle = {
            {style.color[0], style.color[1], style.color[2], style.color[3]},
            {0u, 0u, 0u, 0u},
            textOrigin
        };
        putText(style.label, textStyle);
    }
}

void nvxio::OpenGLRenderImpl::putArrows(vx_array old_points, vx_array new_points,
                                        const LineStyle &line_style)
{
    arrowsRender_.render(old_points, new_points, line_style, wndWidth_, wndHeight_, scaleRatio_);
}

void nvxio::OpenGLRenderImpl::putConvexPoligon(vx_array verticies, const LineStyle& style)
{
    vx_size vCount = 0;
    vxQueryArray(verticies, VX_ARRAY_ATTRIBUTE_NUMITEMS, &vCount, sizeof(vCount));

    if (vCount > 1)
    {
        NVXIO_SAFE_CALL( vxTruncateArray(tmpLines_, 0) );

        vx_size stride = 0;
        vx_coordinates2d_t *ptr = NULL;
        NVXIO_SAFE_CALL( vxAccessArrayRange(verticies, 0, vCount, &stride, (void **)&ptr, VX_READ_ONLY) );

        for (vx_size i = 1; i < vCount; i++)
        {
            nvx_point4f_t line = {
                (vx_float32)ptr[i-1].x, (vx_float32)ptr[i-1].y,
                (vx_float32)ptr[i].x, (vx_float32)ptr[i].y,
            };
            NVXIO_SAFE_CALL( vxAddArrayItems(tmpLines_, 1, &line, sizeof(nvx_point4f_t)) );
        }

        nvx_point4f_t line = {
            (vx_float32)ptr[vCount-1].x, (vx_float32)ptr[vCount-1].y,
            (vx_float32)ptr[0].x, (vx_float32)ptr[0].y,
        };
        NVXIO_SAFE_CALL( vxAddArrayItems(tmpLines_, 1, &line, sizeof(nvx_point4f_t)) );

        NVXIO_SAFE_CALL( vxCommitArrayRange(verticies, 0, vCount, ptr) );

        linesRender_.render(tmpLines_, style, wndWidth_, wndHeight_, scaleRatio_);
    }
}

bool nvxio::OpenGLRenderImpl::initGL(vx_context context,
                                     vx_uint32 wndWidth, vx_uint32 wndHeight,
                                     bool visible)
{
    wndWidth_ = wndWidth;
    wndHeight_ = wndHeight;

    if (!gl_)
    {
        gl_ = std::make_shared<GLFunctions>();
        loadGLFunctions(gl_.get());
    }

    // let's use custom framebuffer in non-visible mode
    if (!visible)
    {
        gl_->GetIntegerv(GL_FRAMEBUFFER_BINDING, (GLint *)&fboOld_);
        NVXIO_CHECK_GL_ERROR();

        gl_->GenTextures(1, &fboTex_);
        NVXIO_CHECK_GL_ERROR();
        gl_->ActiveTexture(GL_TEXTURE0);
        NVXIO_CHECK_GL_ERROR();
        gl_->BindTexture(GL_TEXTURE_2D, fboTex_);
        NVXIO_CHECK_GL_ERROR();

        if (gl_->IsTexture(fboTex_) == GL_FALSE)
        {
            NVXIO_CHECK_GL_ERROR();
            fprintf(stderr, "OpenGL render: failed to create texture object\n");
            return false;
        }

        gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        NVXIO_CHECK_GL_ERROR();
        gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        NVXIO_CHECK_GL_ERROR();
        gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        NVXIO_CHECK_GL_ERROR();
        gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        NVXIO_CHECK_GL_ERROR();
        gl_->TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, wndWidth, wndHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
        NVXIO_CHECK_GL_ERROR();

        gl_->GenFramebuffers(1, &fbo_);
        NVXIO_CHECK_GL_ERROR();
        gl_->BindFramebuffer(GL_FRAMEBUFFER, fbo_);
        NVXIO_CHECK_GL_ERROR();
        if (gl_->IsFramebuffer(fbo_) == GL_FALSE)
        {
            NVXIO_CHECK_GL_ERROR();
            fprintf(stderr, "OpenGL render: failed to create framebuffer object\n");
            return false;
        }

        gl_->FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fboTex_, 0);
        NVXIO_CHECK_GL_ERROR();
        if (gl_->CheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        {
            NVXIO_CHECK_GL_ERROR();
            fprintf(stderr, "OpenGL render: failed to attach framebuffer\n");
            return false;
        }
    }

    if (!imageRender_.init(gl_, wndWidth_, wndHeight_))
        return false;

    if (!featuresRender_.init(gl_))
        return false;

    if (!linesRender_.init(gl_))
        return false;

    if (!motionFieldRender_.init(gl_, wndWidth, wndHeight))
        return false;

    if (!circlesRender_.init(gl_))
        return false;

    if (!textRender_.init(gl_))
        return false;

    if (!rectangleRender_.init(gl_))
        return false;

    if (!arrowsRender_.init(gl_))
        return false;

    tmpLines_ = vxCreateArray(context, NVX_TYPE_POINT4F, 1000);
    NVXIO_CHECK_REFERENCE( tmpLines_ );

    gl_->Enable(GL_BLEND);
    NVXIO_CHECK_GL_ERROR();
    gl_->BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    NVXIO_CHECK_GL_ERROR();

    return true;
}

void nvxio::OpenGLRenderImpl::finalGL()
{
    imageRender_.release();
    featuresRender_.release();
    linesRender_.release();
    motionFieldRender_.release();
    circlesRender_.release();
    textRender_.release();
    rectangleRender_.release();
    arrowsRender_.release();
}

void nvxio::OpenGLRenderImpl::clearGlBuffer()
{
    gl_->ClearColor(0.0f, 0.0, 0.0f, 1.0f);
    NVXIO_CHECK_GL_ERROR();
    gl_->Clear(GL_COLOR_BUFFER_BIT);
    NVXIO_CHECK_GL_ERROR();
}

#endif // USE_GUI
