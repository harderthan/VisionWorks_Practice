/*
 # *Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
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

#if (defined(USE_GSTREAMER) && !defined(GSTREAMERBASEFRAMESOURCEIMPL_HPP))
#define GSTREAMERBASEFRAMESOURCEIMPL_HPP

#include <VX/vx.h>

#include "NVXIO/FrameSource.hpp"
#include "NVX/nvx_timer.hpp"

#include "FrameSource/GStreamer/GStreamerCommon.hpp"

// GStreamer pipeline
enum GstAutoplugSelectResult
{
    GST_AUTOPLUG_SELECT_TRY,
    GST_AUTOPLUG_SELECT_EXPOSE,
    GST_AUTOPLUG_SELECT_SKIP
};

namespace nvxio
{

class GStreamerBaseFrameSourceImpl : public nvxio::FrameSource
{
public:
    GStreamerBaseFrameSourceImpl(vx_context context, FrameSource::SourceType, const std::string & name);
    virtual bool open();
    virtual FrameSource::FrameStatus fetch(vx_image image,
                                           vx_uint32 timeout = 5 /*milliseconds*/);
    virtual FrameSource::Parameters getConfiguration();
    virtual bool setConfiguration(const FrameSource::Parameters& params);
    virtual void close();
    virtual ~GStreamerBaseFrameSourceImpl();

    static void newGstreamerPad(GstElement * /*elem*/, GstPad *pad, gpointer data);

protected:

    GStreamerBaseFrameSourceImpl();

    void handleGStreamerMessages();

    virtual bool InitializeGstPipeLine() = 0;
    void FinalizeGstPipeLine();

    GstPipeline*  pipeline;
    GstBus*       bus;

    volatile bool end;

    // Common FrameSource parameters
    vx_context vxContext;
    FrameSource::Parameters configuration;
    nvx::Timer lastFrameTimestamp;

    GstElement * sink;

private:
    // temporary CUDA buffer
    void * devMem;
    size_t devMemPitch;

    vx_image scaledImage;
};

}

#endif // GSTREAMERBASEFRAMESOURCEIMPL_HPP
