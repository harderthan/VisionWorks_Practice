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

#include "FrameSource/OpenCV/OpenCVImageFrameSource.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nvxio
{

OpenCVImageFrameSource::OpenCVImageFrameSource(const std::string& _fileName):
    OpenCVBaseFrameSource(FrameSource::SINGLE_IMAGE_SOURCE, "OpenCVImageSource"),
    fileName(_fileName),
    opened(false)
{
}

bool OpenCVImageFrameSource::open()
{
    bool result = true;
    cv::Mat orig;

    try
    {
        switch(configuration.format)
        {
        case VX_DF_IMAGE_U8:
            image = cv::imread(fileName, cv::IMREAD_GRAYSCALE);
            break;
        case VX_DF_IMAGE_RGB:
            orig = cv::imread(fileName, cv::IMREAD_COLOR);
            if (!orig.empty())
                cv::cvtColor(orig, image, CV_BGR2RGB);
            else
                result = false;
            break;
        case VX_DF_IMAGE_RGBX:
            orig = cv::imread(fileName, cv::IMREAD_ANYCOLOR);
            if (orig.empty())
            {
                result = false;
                break;
            }

            switch (orig.type())
            {
            case CV_8UC1:
                image = orig;
                break;
            case CV_8UC3:
                cv::cvtColor(orig, image, CV_BGR2RGBA);
                break;
            case CV_8UC4:
                image = orig;
                break;
            default:
                result = false;
            }
            break;
        default:
            result = false;
        }
    }
    catch(...)
    {
        result = false;
    }

    opened = result && !image.empty();

    if (opened)
    {
        configuration.frameWidth = image.cols;
        configuration.frameHeight = image.rows;
    }

    return opened;
}

bool OpenCVImageFrameSource::setConfiguration(const FrameSource::Parameters& params)
{
    configuration.format = params.format;
    return true;
}

FrameSource::Parameters OpenCVImageFrameSource::getConfiguration()
{
    return configuration;
}

cv::Mat OpenCVImageFrameSource::fetch()
{
    opened  = false;
    return image;
}

bool OpenCVImageFrameSource::grab()
{
    return opened;
}

void OpenCVImageFrameSource::close()
{
    opened = false;
}

OpenCVImageFrameSource::~OpenCVImageFrameSource()
{
    close();
}

}

#endif
