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

#ifndef OPENCVVIDEOFRAMESOURCE_HPP
#define OPENCVVIDEOFRAMESOURCE_HPP

#ifdef USE_OPENCV

#include "FrameSource/OpenCV/OpenCVBaseFrameSource.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nvxio
{

class OpenCVVideoFrameSource : public nvxio::OpenCVBaseFrameSource
{
public:
    explicit OpenCVVideoFrameSource(int _cameraId);
    OpenCVVideoFrameSource(const std::string& _fileName, bool sequence);

    virtual bool open();
    virtual bool setConfiguration(const FrameSource::Parameters& params);
    virtual FrameSource::Parameters getConfiguration();
    virtual cv::Mat fetch();
    virtual bool grab();
    virtual void close();
    virtual ~OpenCVVideoFrameSource();
protected:
    void updateConfiguration();

    std::string fileName;
    int cameraId;
    FrameSource::Parameters configuration;
    cv::VideoCapture capture;
};

}

#endif // USE_OPENCV
#endif // OPENCVVIDEOFRAMESOURCE_HPP