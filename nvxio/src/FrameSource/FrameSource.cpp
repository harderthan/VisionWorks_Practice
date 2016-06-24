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
# include "FrameSource/OpenCV/OpenCVFrameSourceImpl.hpp"
# include "FrameSource/OpenCV/OpenCVImageFrameSource.hpp"
# include "FrameSource/OpenCV/OpenCVVideoFrameSource.hpp"
#endif

#ifdef USE_GSTREAMER
# if defined(USE_NVMEDIA) && defined(USE_HARDWARE_CODEC)
#  include "FrameSource/GStreamer/GStreamerNvMediaFrameSourceImpl.hpp"
# else
#  include "FrameSource/GStreamer/GStreamerVideoFrameSourceImpl.hpp"
# endif
# include "FrameSource/GStreamer/GStreamerCameraFrameSourceImpl.hpp"
# include "FrameSource/GStreamer/GStreamerImagesFrameSourceImpl.hpp"
# ifdef USE_NVGSTCAMERA
#  include "FrameSource/GStreamer/GStreamerNvCameraFrameSourceImpl.hpp"
# endif
# if defined USE_GSTREAMER_OMX && defined USE_GLES // For L4T R23 only
#  include "FrameSource/GStreamer/GStreamerOpenMAXFrameSourceImpl.hpp"
# endif
#endif

#ifdef USE_NVMEDIA
# include "FrameSource/NvMediaFrameSourceImpl.hpp"
#endif

#include "NVXIO/Utility.hpp"
#include "NVXIO/ThreadSafeQueue.hpp"
#include "NVXIO/FrameSource.hpp"

namespace nvxio
{

static std::string resolveFileUri(const std::string& uri)
{
    return uri.substr(7);
}

static int resolveDeviceUri(const std::string & uri, bool & isNvCamera)
{
    int idx = -1;
    isNvCamera = false;

    if (sscanf(uri.c_str(), "device://nvcamera%d", &idx) == 1)
        isNvCamera = true;
    else if (sscanf(uri.c_str(), "device://camera%d", &idx) == 1)
        isNvCamera = false;

    return idx;
}

std::unique_ptr<FrameSource> createDefaultFrameSource(vx_context context, const std::string& uri)
{
    if (uri.empty())
        return nullptr;

    size_t prefixLength = uri.find("://");
    std::string protocol;
    if (prefixLength != std::string::npos)
    {
        protocol = uri.substr(0, prefixLength + 3);
    }

    if (protocol.empty() || protocol == "file://")
    {
        std::string path = protocol.empty() ? uri : resolveFileUri(uri);

        if (!path.empty())
        {
            std::string ext = path.substr(path.rfind(".") + 1);
            if ((ext == std::string("png")) ||
                (ext == std::string("jpg")) ||
                (ext == std::string("jpeg")) ||
                (ext == std::string("bmp")) ||
                (ext == std::string("tiff")))
            {
                size_t pos = path.find('%');
                bool isImageSequence = pos != std::string::npos;

#ifdef USE_GSTREAMER
                return makeUP<GStreamerImagesFrameSourceImpl>(context, isImageSequence ? FrameSource::IMAGE_SEQUENCE_SOURCE :
                                                                                         FrameSource::SINGLE_IMAGE_SOURCE, uri);

#endif
#ifdef USE_OPENCV
                std::unique_ptr<OpenCVBaseFrameSource> ocvSource;

                if (isImageSequence)
                    ocvSource.reset(new OpenCVVideoFrameSource(path, true));
                else
                    ocvSource.reset(new OpenCVImageFrameSource(path));

                if (ocvSource)
                    return makeUP<OpenCVFrameSourceImpl>(context, std::move(ocvSource));
#endif
            }
            else
            {
#ifdef USE_NVMEDIA
                if (ext == std::string("h264"))
                    return makeUP<NvMediaFrameSourceImpl>(context, path);
#endif
#if defined(USE_GSTREAMER)
# if defined(USE_NVMEDIA) && defined(USE_HARDWARE_CODEC)
                return makeUP<GStreamerNvMediaFrameSourceImpl>(context, path);
# else
#  if defined USE_GSTREAMER_OMX && defined USE_GLES // For L4T R23 only
                return makeUP<GStreamerOpenMAXFrameSourceImpl>(context, path);
#  else
                return makeUP<GStreamerVideoFrameSourceImpl>(context, path);
#  endif
# endif
#else
                (void)context;
#ifdef USE_OPENCV
                std::unique_ptr<OpenCVVideoFrameSource> ocvSource(new OpenCVVideoFrameSource(path, false));
                if (ocvSource)
                    return makeUP<OpenCVFrameSourceImpl>(context, std::move(ocvSource));
#endif
#endif
            }
        }
    }
    else if (protocol == "device://")
    {
#if defined USE_GSTREAMER || defined USE_OPENCV
        bool isNvCamera = false;
        int idx = resolveDeviceUri(uri, isNvCamera);

        if (idx >= 0)
        {
#ifdef USE_GSTREAMER
# ifdef USE_NVGSTCAMERA
            if (isNvCamera)
                return makeUP<GStreamerNvCameraFrameSourceImpl>(context, static_cast<uint>(idx));
# endif
            if (isNvCamera)
            {
                printf("NvCamera source is not available on this platform\n");
                return nullptr;
            }

            return makeUP<GStreamerCameraFrameSourceImpl>(context, static_cast<uint>(idx));
#endif
#ifdef USE_OPENCV
            std::unique_ptr<OpenCVVideoFrameSource> ocvSource(new OpenCVVideoFrameSource(idx));
            if (ocvSource)
                return makeUP<OpenCVFrameSourceImpl>(context, std::move(ocvSource));
#endif
        }
#endif
    }

    return nullptr;
}

vx_image loadImageFromFile(vx_context context, const std::string& fileName, vx_df_image format)
{
    auto frameSource = createDefaultFrameSource(context, fileName);
    if (!frameSource)
    {
        NVXIO_THROW_EXCEPTION("Cannot create frame source for file: " << fileName);
    }

    if (frameSource->getSourceType() != FrameSource::SINGLE_IMAGE_SOURCE)
    {
        NVXIO_THROW_EXCEPTION("Expected " << fileName << " to be an image");
    }

    auto frameConfig = frameSource->getConfiguration();
    frameConfig.format = format;
    frameSource->setConfiguration(frameConfig);

    if (!frameSource->open())
    {
        NVXIO_THROW_EXCEPTION("Cannot open file: " << fileName);
    }

    frameConfig = frameSource->getConfiguration();

    vx_image image = vxCreateImage(context, frameConfig.frameWidth, frameConfig.frameHeight, format);
    NVXIO_CHECK_REFERENCE(image);

    if (frameSource->fetch(image, TIMEOUT_INFINITE) != FrameSource::OK)
    {
        vxReleaseImage(&image);
        NVXIO_THROW_EXCEPTION("Cannot fetch a frame from file: " << fileName);
    }

    return image;
}

}
