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

#ifdef USE_OPENCV

#include "FrameSource/OpenCV/OpenCVVideoFrameSource.hpp"

#include <opencv2/imgproc/imgproc.hpp>
#include <NVXIO/Utility.hpp>

namespace nvxio
{

OpenCVVideoFrameSource::OpenCVVideoFrameSource(int _cameraId):
    OpenCVBaseFrameSource(FrameSource::CAMERA_SOURCE, "OpenCVVideoFrameSource"),
    fileName(),
    cameraId(_cameraId)
{
}

OpenCVVideoFrameSource::OpenCVVideoFrameSource(const std::string& _fileName, bool sequence):
    OpenCVBaseFrameSource(sequence ? FrameSource::IMAGE_SEQUENCE_SOURCE : FrameSource::VIDEO_SOURCE,
                 "OpenCVVideoFrameSource"),
    fileName(_fileName),
    cameraId(-1)
{
}

bool OpenCVVideoFrameSource::open()
{
    bool opened = false;

    FrameSource::Parameters tmp = configuration;
    setConfiguration(tmp);

    if (fileName.empty())
        opened = capture.open(cameraId);
    else
        opened = capture.open(fileName);

    if (opened)
        updateConfiguration();

    return opened;
}

bool OpenCVVideoFrameSource::setConfiguration(const FrameSource::Parameters& params)
{
    bool result = true;

    if ((params.frameWidth != (vx_uint32)-1) && (params.frameWidth != configuration.frameWidth))
        result &= capture.set(CV_CAP_PROP_FRAME_WIDTH, params.frameWidth);
    if ((params.frameHeight != (vx_uint32)-1) && (params.frameHeight != configuration.frameHeight))
        result &= capture.set(CV_CAP_PROP_FRAME_HEIGHT, params.frameHeight);
    if ((params.fps != (vx_uint32)-1) && (params.fps != configuration.fps))
        result &= capture.set(CV_CAP_PROP_FPS, params.fps);
    configuration.format = params.format;

    updateConfiguration();

    return result;
}

void OpenCVVideoFrameSource::updateConfiguration()
{
    configuration.fps = capture.get(CV_CAP_PROP_FPS);
    configuration.frameWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH);
    configuration.frameHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT);
}

FrameSource::Parameters OpenCVVideoFrameSource::getConfiguration()
{
    return configuration;
}

cv::Mat OpenCVVideoFrameSource::fetch()
{
    cv::Mat orig;
    cv::Mat image;

    capture.retrieve(orig);

    switch(configuration.format)
    {
    case VX_DF_IMAGE_U8:
        cv::cvtColor(orig, image, CV_BGR2GRAY);
        break;
    case VX_DF_IMAGE_RGB:
        cv::cvtColor(orig, image, CV_BGR2RGB);
        break;
    case VX_DF_IMAGE_RGBX:
        cv::cvtColor(orig, image, CV_BGR2RGBA);
        break;
    default:
        NVXIO_THROW_EXCEPTION("Unsupported format\n");
    }

    return image;
}

bool OpenCVVideoFrameSource::grab()
{
    return capture.grab();
}

void OpenCVVideoFrameSource::close()
{
    capture.release();
}

OpenCVVideoFrameSource::~OpenCVVideoFrameSource()
{
    close();
}

}

#endif
