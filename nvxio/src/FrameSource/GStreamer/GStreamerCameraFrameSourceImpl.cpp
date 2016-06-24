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

#ifdef USE_GSTREAMER

#include "FrameSource/GStreamer/GStreamerCameraFrameSourceImpl.hpp"

#include <gst/app/gstappsink.h>

#include <sstream>

namespace nvxio
{

GStreamerCameraFrameSourceImpl::GStreamerCameraFrameSourceImpl(vx_context context, uint cameraIdx_) :
    GStreamerBaseFrameSourceImpl(context, FrameSource::VIDEO_SOURCE, "GstreamerCameraFrameSource"),
    cameraIdx(cameraIdx_)
{
}

bool GStreamerCameraFrameSourceImpl::setConfiguration(const FrameSource::Parameters& params)
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


bool GStreamerCameraFrameSourceImpl::InitializeGstPipeLine()
{
    GstStateChangeReturn status;
    end = true;

    pipeline = GST_PIPELINE(gst_pipeline_new(NULL));
    if (pipeline == NULL)
    {
        printf("Cannot create Gstreamer pipeline\n");
        return false;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));

    // create v4l2src
    GstElement * v4l2src = gst_element_factory_make("v4l2src", NULL);
    if (v4l2src == NULL)
    {
        printf("Cannot create v4l2src\n");
        FinalizeGstPipeLine();

        return false;
    }

    std::ostringstream cameraDev;
    cameraDev << "/dev/video" << cameraIdx;
    g_object_set(G_OBJECT(v4l2src), "device", cameraDev.str().c_str(), NULL);

    gst_bin_add(GST_BIN(pipeline), v4l2src);

    // create color convert element
    GstElement * color = gst_element_factory_make(COLOR_ELEM, NULL);
    if (color == NULL)
    {
        printf("Cannot create %s element\n", COLOR_ELEM);
        FinalizeGstPipeLine();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), color);

    // create appsink element
    sink = gst_element_factory_make("appsink", NULL);
    if (sink == NULL)
    {
        printf("Cannot create appsink element\n");
        FinalizeGstPipeLine();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), sink);

    // if initial values for FrameSource::Parameters are not
    // specified, let's set them manually to prevent very huge images
    if (configuration.frameWidth == (vx_uint32)-1)
        configuration.frameWidth = 1920;
    if (configuration.frameHeight == (vx_uint32)-1)
        configuration.frameHeight = 1080;
    if (configuration.fps == (vx_uint32)-1)
        configuration.fps = 30;

#if GST_VERSION_MAJOR == 0
    GstCaps* caps_v42lsrc = gst_caps_new_simple ("video/x-raw-rgb",
                 "width", GST_TYPE_INT_RANGE, 1, (int)configuration.frameWidth,
                 "height", GST_TYPE_INT_RANGE, 1, (int)configuration.frameHeight,
                 "framerate", GST_TYPE_FRACTION, (int)configuration.fps,
                 NULL);
#else
    std::ostringstream stream;
    stream << "video/x-raw, format=(string){RGB, GRAY8}, width=[1," << configuration.frameWidth <<
              "], height=[1," << configuration.frameHeight << "], framerate=" << configuration.fps << "/1;";

    GstCaps* caps_v42lsrc = gst_caps_from_string(stream.str().c_str());
#endif

    if (caps_v42lsrc == NULL)
    {
        printf("Failed to create caps\n");
        FinalizeGstPipeLine();

        return false;
    }

    // link elements
    if (!gst_element_link_filtered(v4l2src, color, caps_v42lsrc))
    {
        printf("GStreamer: cannot link v4l2src -> color using caps\n");
        FinalizeGstPipeLine();
        gst_caps_unref(caps_v42lsrc);

        return false;
    }
    gst_caps_unref(caps_v42lsrc);

    // link elements
    if (!gst_element_link(color, sink))
    {
        printf("GStreamer: cannot link color -> appsink\n");
        FinalizeGstPipeLine();

        return false;
    }

    gst_app_sink_set_max_buffers (GST_APP_SINK(sink), 1);
    gst_app_sink_set_drop (GST_APP_SINK(sink), true);

    // do not emit signals: all calls will be synchronous and blocking
    gst_app_sink_set_emit_signals (GST_APP_SINK(sink), 0);

#if GST_VERSION_MAJOR == 0
    GstCaps* caps_appsink = gst_caps_new_simple("video/x-raw-rgb",
                                                "bpp",        G_TYPE_INT, 24,
                                                "red_mask",   G_TYPE_INT, 0xFF0000,
                                                "green_mask", G_TYPE_INT, 0x00FF00,
                                                "blue_mask",  G_TYPE_INT, 0x0000FF,
                                                NULL);
#else
    // support 1 and 3 channel 8 bit data
    GstCaps* caps_appsink = gst_caps_from_string("video/x-raw, format=(string){RGB, GRAY8};");
#endif
    gst_app_sink_set_caps(GST_APP_SINK(sink), caps_appsink);
    gst_caps_unref(caps_appsink);

    // Force pipeline to play video as fast as possible, ignoring system clock
    gst_pipeline_use_clock(pipeline, NULL);

    status = gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
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

    std::unique_ptr<GstPad, GStreamerObjectDeleter> pad(gst_element_get_static_pad(color, "src"));
#if GST_VERSION_MAJOR == 0
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> bufferCaps(gst_pad_get_caps(pad.get()));
#else
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> bufferCaps(gst_pad_get_current_caps(pad.get()));
#endif

    const GstStructure *structure = gst_caps_get_structure(bufferCaps.get(), 0);

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

}

#endif
