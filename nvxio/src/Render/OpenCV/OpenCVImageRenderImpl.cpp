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

#include "Render/OpenCV/OpenCVImageRenderImpl.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

namespace nvxio
{

OpenCVImageRenderImpl::OpenCVImageRenderImpl():
    OpenCVBaseRenderImpl(nvxio::Render::IMAGE_RENDER, "OpenCVImageWriter"),
    keyboardCallback(NULL),
    mouseCallback(NULL),
    currentFrameIdx(-1)
{
}

bool OpenCVImageRenderImpl::open(const std::string &path, vx_uint32 width, vx_uint32 height, vx_uint32 /*format*/)
{
    if (path.find("%", 0) == std::string::npos)
        return false;

    currentFrameIdx = 0;
    windowTitle = path;
    windowResolution = cv::Size(width, height);

    resizeBuffers(width, height);

    return true;
}

bool OpenCVImageRenderImpl::flush()
{
    if (windowTitle.empty() || currentFrameIdx < 0)
        return false;

    composeFrame();

    // Use OpenCV's BGRA color format instead of VisionWorks' RGBA format.
    // Supports transparent PNGs as well as images that were already
    // in BGR or Grayscale format.
    cv::Mat matBGRA = displayFrame;

    if (displayFrame.channels() == 4)
    {
        // Convert from VisionWorks RGBA to OpenCV BGRA color format.
        cv::cvtColor(displayFrame, displayFrameBGRA, CV_RGBA2BGRA);
        matBGRA = displayFrameBGRA;   // Use the new image
    }

    std::string fileName = cv::format(windowTitle.c_str(), currentFrameIdx);
    currentFrameIdx++;

    return cv::imwrite(fileName, matBGRA);
}

bool OpenCVImageRenderImpl::resizeBuffers(vx_uint32 width, vx_uint32 height)
{
    if ((width == 0) || (height == 0))
        return false;

    textureResolution = cv::Size(width, height);
    displayFrame = cv::Mat(textureResolution.height, textureResolution.width, CV_8UC4);
    displayFrameBGRA = cv::Mat(textureResolution.height, textureResolution.width, CV_8UC4);

    return true;
}

void OpenCVImageRenderImpl::close()
{
    currentFrameIdx = -1;
}

OpenCVImageRenderImpl::~OpenCVImageRenderImpl()
{
    close();
}

void OpenCVImageRenderImpl::setOnKeyboardEventCallback(Render::OnKeyboardEventCallback callback, void* context)
{
    keyboardCallback = callback;
    onKeyboardCallbackContext_ = context;
}

void OpenCVImageRenderImpl::setOnMouseEventCallback(Render::OnMouseEventCallback callback, void* context)
{
    mouseCallback = callback;
    onMouseCallbackContext_ = context;
}

}

#endif
