/*
# Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#ifdef USE_OPENCV

#include "Render/OpenCV/OpenCVVideoRenderImpl.hpp"
#include "NVXIO/Render.hpp"
#include <opencv2/imgproc/imgproc.hpp>

namespace nvxio
{

OpenCVVideoRenderImpl::OpenCVVideoRenderImpl():
    OpenCVBaseRenderImpl(nvxio::Render::VIDEO_RENDER, "OpenCVVideoWriter"),
    keyboardCallback(NULL),
    mouseCallback(NULL)
{
}

bool OpenCVVideoRenderImpl::open(const std::string &path, vx_uint32 width, vx_uint32 height, vx_uint32 format)
{
    (void)format;
    windowTitle = path;
    windowResolution = cv::Size(width, height);
    resizeBuffers(width, height);

    return writer.open(path, CV_FOURCC('m', 'p', '4', 'v'), 30, windowResolution);
}

bool OpenCVVideoRenderImpl::resizeBuffers(vx_uint32 width, vx_uint32 height)
{
    if ((width == 0) || (height == 0))
        return false;

    textureResolution = cv::Size(width, height);
    displayFrame = cv::Mat(textureResolution.height, textureResolution.width, CV_8UC4);

    return true;
}

bool OpenCVVideoRenderImpl::flush()
{
    composeFrame();

    cv::cvtColor(displayFrame, displayFrameBGR, CV_RGBA2BGR);
    if ((displayFrameBGR.cols == windowResolution.width) && (displayFrameBGR.rows == windowResolution.height))
    {
        writer.write(displayFrameBGR);
    }
    else
    {
        cv::resize(displayFrameBGR, displayFrameResized, windowResolution);
        writer.write(displayFrameResized);
    }

    return true;
}

void OpenCVVideoRenderImpl::close()
{
    writer.release();
}

OpenCVVideoRenderImpl::~OpenCVVideoRenderImpl()
{
    close();
}

void OpenCVVideoRenderImpl::setOnKeyboardEventCallback(Render::OnKeyboardEventCallback callback, void* context)
{
    keyboardCallback = callback;
    onKeyboardCallbackContext_ = context;
}

void OpenCVVideoRenderImpl::setOnMouseEventCallback(Render::OnMouseEventCallback callback, void* context)
{
    mouseCallback = callback;
    onMouseCallbackContext_ = context;
}

}

#endif
