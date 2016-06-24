/*
 # *Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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

#ifdef USE_NVGSTCAMERA

#include "GStreamerNvCameraFrameSourceImpl.hpp"

#include <gst/app/gstappsink.h>
#include <sstream>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "NVXIO/Utility.hpp"
#include "NVXIO/Application.hpp"

namespace nvxio
{

struct NvCameraConfigs
{
    vx_uint32 frameWidth, frameHeight, fps;
};

static const NvCameraConfigs configs[4] =
{
    { vx_uint32(2592), vx_uint32(1944), vx_uint32(30)  }, // 0
    { vx_uint32(2592), vx_uint32(1458), vx_uint32(30)  }, // 1
    { vx_uint32(1280), vx_uint32(720) , vx_uint32(120) }, // 2
    { vx_uint32(2592), vx_uint32(1944), vx_uint32(24)  }  // 3
};

GStreamerNvCameraFrameSourceImpl::GStreamerNvCameraFrameSourceImpl(vx_context vxcontext, uint cameraIdx_) :
    GStreamerEGLStreamSinkFrameSourceImpl(vxcontext, FrameSource::CAMERA_SOURCE, "GStreamerNvCameraFrameSource", false),
    cameraIdx(cameraIdx_)
{
}

bool GStreamerNvCameraFrameSourceImpl::setConfiguration(const FrameSource::Parameters& params)
{
    bool result = true;

    if (end)
    {
        configuration.frameHeight = params.frameHeight;
        configuration.frameWidth = params.frameWidth;
        configuration.fps = params.fps;
    }
    else
    {
        if ((params.frameWidth != (vx_uint32)-1) && (params.frameWidth != configuration.frameWidth))
            result = false;
        if ((params.frameHeight != (vx_uint32)-1) && (params.frameHeight != configuration.frameHeight))
            result = false;
        if ((params.fps != (vx_uint32)-1) && (params.fps != configuration.fps))
            result = false;
    }

    configuration.format = params.format;

    return result;
}

bool GStreamerNvCameraFrameSourceImpl::InitializeGstPipeLine()
{
    // select config with max FPS value to be default
    NvCameraConfigs nvcameraconfig = configs[2];

    // use user specified camera config
    if ( (configuration.frameWidth != (vx_uint32)-1) &&
         (configuration.frameHeight != (vx_uint32)-1) )
    {
        nvcameraconfig.frameWidth = configuration.frameWidth;
        nvcameraconfig.frameHeight = configuration.frameHeight;
        nvcameraconfig.fps = 30;

        // select FPS default for the specified config
        for (vx_size i = 0; i < dimOf(configs); ++i)
        {
            if ((nvcameraconfig.frameWidth == configs[i].frameWidth) &&
                (nvcameraconfig.frameHeight == configs[i].frameHeight))
            {
                nvcameraconfig.fps = configs[i].fps;
                break;
            }
        }
    }

    if (configuration.fps == (vx_uint32)-1)
        configuration.fps = nvcameraconfig.fps;

    end = true;

    pipeline = GST_PIPELINE(gst_pipeline_new(NULL));
    if (pipeline == NULL)
    {
        printf("Cannot create Gstreamer pipeline\n");
        return false;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));

    // create nvcamerasrc
    GstElement * nvcamerasrc = gst_element_factory_make("nvcamerasrc", NULL);
    if (nvcamerasrc == NULL)
    {
        printf("Cannot create nvcamerasrc\n\"nvcamerasrc\" element is not available on this platform\n");
        FinalizeGstPipeLine();

        return false;
    }

    std::ostringstream stream;
    stream << configuration.fps << " " << configuration.fps;
    std::string fpsRange = stream.str();

    g_object_set(G_OBJECT(nvcamerasrc), "sensor-id", cameraIdx, NULL);
    g_object_set(G_OBJECT(nvcamerasrc), "fpsRange", fpsRange.c_str(), NULL);

    gst_bin_add(GST_BIN(pipeline), nvcamerasrc);

    // create nvvideosink element
    GstElement * nvvideosink = gst_element_factory_make("nvvideosink", NULL);
    if (nvvideosink == NULL)
    {
        printf("Cannot create nvvideosink element\n");
        FinalizeGstPipeLine();

        return false;
    }

    g_object_set(G_OBJECT(nvvideosink), "display", context.display, NULL);
    g_object_set(G_OBJECT(nvvideosink), "stream", context.stream, NULL);
    g_object_set(G_OBJECT(nvvideosink), "fifo", TRUE, NULL);
    g_object_set(G_OBJECT(nvvideosink), "max-lateness", -1, NULL);
    g_object_set(G_OBJECT(nvvideosink), "throttle-time", 0, NULL);
    g_object_set(G_OBJECT(nvvideosink), "render-delay", 0, NULL);
    g_object_set(G_OBJECT(nvvideosink), "qos", FALSE, NULL);
    g_object_set(G_OBJECT(nvvideosink), "sync", FALSE, NULL);
    g_object_set(G_OBJECT(nvvideosink), "async", TRUE, NULL);

    gst_bin_add(GST_BIN(pipeline), nvvideosink);

    // link elements
    stream.str(std::string());
    stream << "video/x-raw(memory:NVMM), width=(int)" << nvcameraconfig.frameWidth << ", "
              "height=(int)" << nvcameraconfig.frameHeight << ", format=(string){I420}, "
              "framerate=(fraction)" << nvcameraconfig.fps << "/1;";

    std::unique_ptr<GstCaps, GStreamerObjectDeleter> caps_nvvidconv(
        gst_caps_from_string(stream.str().c_str()));

    if (!caps_nvvidconv)
    {
        printf("Failed to create caps\n");
        FinalizeGstPipeLine();

        return false;
    }

    if (!gst_element_link_filtered(nvcamerasrc, nvvideosink, caps_nvvidconv.get()))
    {
        printf("GStreamer: cannot link nvvidconv -> nvvideosink using caps\n");
        FinalizeGstPipeLine();

        return false;
    }

    // Force pipeline to play video as fast as possible, ignoring system clock
    gst_pipeline_use_clock(pipeline, NULL);

    GstStateChangeReturn status = gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
    handleGStreamerMessages();

    if (status == GST_STATE_CHANGE_ASYNC)
    {
        // wait for status update
        status = gst_element_get_state(GST_ELEMENT(pipeline), NULL, NULL, GST_CLOCK_TIME_NONE);
    }
    if (status == GST_STATE_CHANGE_FAILURE)
    {
        printf("GStreamer: unable to start playback\n");
        FinalizeGstPipeLine();

        return false;
    }

    std::unique_ptr<GstPad, GStreamerObjectDeleter> pad(gst_element_get_static_pad(nvvideosink, "sink"));
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> buffer_caps(gst_pad_get_current_caps(pad.get()));

    const GstStructure *structure = gst_caps_get_structure(buffer_caps.get(), 0);

    int width, height;
    if (!gst_structure_get_int(structure, "width", &width))
    {
        handleGStreamerMessages();
        printf("Cannot query video width\n");
    }

    if (!gst_structure_get_int(structure, "height", &height))
    {
        handleGStreamerMessages();
        printf("Cannot query video height\n");
    }

    configuration.frameWidth = static_cast<vx_uint32>(width);
    configuration.frameHeight = static_cast<vx_uint32>(height);

    gint num = 0, denom = 1;
    if (!gst_structure_get_fraction(structure, "framerate", &num, &denom))
    {
        handleGStreamerMessages();
        printf("Cannot query video fps\n");
    }

    configuration.fps = static_cast<float>(num) / denom;
    end = false;

    return true;
}

} // namespace nvxio

#endif // USE_NVGSTCAMERA
