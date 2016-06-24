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
#include <system_error>

#include "FrameSource/OpenCV/OpenCVFrameSourceImpl.hpp"

#include "NVX/nvx.h"
#include <cuda_runtime.h>
#include "NVXIO/Application.hpp"
#include "NVX/nvx_opencv_interop.hpp"
#include <map>

namespace nvxio
{

std::map<int,unsigned int> OCV2VX =  {{CV_8UC1, VX_DF_IMAGE_U8}, {CV_8UC3, VX_DF_IMAGE_RGB},{CV_8UC4, VX_DF_IMAGE_RGBX}};

OpenCVFrameSourceImpl::OpenCVFrameSourceImpl(vx_context context, std::unique_ptr<OpenCVBaseFrameSource> source):
    FrameSource(source->getSourceType(), source->getSourceName()),
    alive_(false),
    source_(std::move(source)),
    queue_(4),
    context_(context)
{
}

bool OpenCVFrameSourceImpl::open()
{
    if (source_ == NULL)
        return false;

    if (alive_)
        close();

    alive_ = source_->open();
    if (alive_)
    {
        try
        {
            thread = std::thread(&OpenCVFrameSourceImpl::threadFunc, this);
            lastFrameTimestamp.tic();
            return true;
        }
        catch (std::system_error &)
        {
            alive_ = false;
            source_->close();
        }
    }

    return alive_;
}

FrameSource::Parameters OpenCVFrameSourceImpl::getConfiguration()
{
    return source_->getConfiguration();
}

bool OpenCVFrameSourceImpl::setConfiguration(const FrameSource::Parameters &params)
{
    return source_->setConfiguration(params);
}

FrameSource::FrameStatus OpenCVFrameSourceImpl::fetch(vx_image image, vx_uint32 timeout)
{
    cv::Mat frame;
    if (queue_.pop(frame, timeout))
    {
        vx_imagepatch_addressing_t image_addr;
        image_addr.dim_x = frame.cols;
        image_addr.dim_y = frame.rows;
        image_addr.stride_x = frame.elemSize();
        image_addr.stride_y = frame.step;

        void *image_ptr[] = {
            frame.data
        };

        vx_image vxframe = vxCreateImageFromHandle(context_, OCV2VX[frame.type()], &image_addr, image_ptr, VX_IMPORT_TYPE_HOST);
        vx_status status;
        if(OCV2VX[frame.type()]==source_->getConfiguration().format)
        {
            status = nvxuCopyImage(context_,vxframe,image);
        }
        else
        {
            status = vxuColorConvert(context_,vxframe,image);
        }

        if (status != VX_SUCCESS)
        {
            close();
            return CLOSED;
        }
        vxReleaseImage(&vxframe);

        lastFrameTimestamp.tic();

        return OK;
    }
    else
    {
        if (alive_)
        {
            if ((lastFrameTimestamp.toc()/1000.0) > Application::get().getSourceDefaultTimeout())
            {
                close();
                return CLOSED;
            }
            else
                return TIMEOUT;
        }
        else
        {
            close();
            return CLOSED;
        }
    }
}

void OpenCVFrameSourceImpl::close()
{
    alive_ = false;
    if (thread.joinable())
        thread.join();
    queue_.clear();
    source_->close();
}

OpenCVFrameSourceImpl::~OpenCVFrameSourceImpl()
{
    close();
}

void OpenCVFrameSourceImpl::threadFunc()
{
    const unsigned int timeout = 30; /*milliseconds*/

    while (alive_ && source_->grab())
    {
        cv::Mat tmp = source_->fetch();
        while (alive_ && !queue_.push(tmp, timeout)) {};
    }

    alive_ = false;
}

}

#endif
