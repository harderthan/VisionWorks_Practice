/*
# Copyright (c) 2015, NVIDIA CORPORATION. All rights reserved.
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

#if defined USE_GUI && defined USE_GSTREAMER

#include "Render/GStreamer/GStreamerVideoRenderImpl.hpp"

nvxio::GStreamerVideoRenderImpl::GStreamerVideoRenderImpl(vx_context context) :
    GStreamerBaseRenderImpl(context, nvxio::Render::VIDEO_RENDER, "GStreamerVideoOpenGlRender")
{
}

bool nvxio::GStreamerVideoRenderImpl::InitializeGStreamerPipeline()
{
    std::ostringstream stream;

    pipeline = GST_PIPELINE(gst_pipeline_new(NULL));
    if (pipeline == NULL)
    {
        printf("Cannot create Gstreamer pipeline\n");
        return false;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));

    // create appsrc
    GstElement * appsrcelem = gst_element_factory_make("appsrc", NULL);
    if (appsrcelem == NULL)
    {
        printf("Cannot create appsrc\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    g_object_set(G_OBJECT(appsrcelem), "is-live", 0, NULL);
    g_object_set(G_OBJECT(appsrcelem), "num-buffers", -1, NULL);
    g_object_set(G_OBJECT(appsrcelem), "emit-signals", 0, NULL);
    g_object_set(G_OBJECT(appsrcelem), "block", 1, NULL);
    g_object_set(G_OBJECT(appsrcelem), "size", static_cast<guint64>(wndHeight_ * wndWidth_ * 4), NULL);
    g_object_set(G_OBJECT(appsrcelem), "format", GST_FORMAT_TIME, NULL);
    g_object_set(G_OBJECT(appsrcelem), "stream-type", GST_APP_STREAM_TYPE_STREAM, NULL);

    appsrc = GST_APP_SRC_CAST(appsrcelem);
#if GST_VERSION_MAJOR == 0
    GstCaps * caps = gst_caps_new_simple("video/x-raw-rgb",
                                         "bpp", G_TYPE_INT, 32,
                                         "endianness", G_TYPE_INT, 4321,
                                         "red_mask", G_TYPE_INT, -16777216,
                                         "green_mask", G_TYPE_INT, 16711680,
                                         "blue_mask", G_TYPE_INT, 65280,
                                         "alpha_mask", G_TYPE_INT, 255,
                                         "width", G_TYPE_INT, wndWidth_,
                                         "height", G_TYPE_INT, wndHeight_,
                                         "framerate", GST_TYPE_FRACTION, GSTREAMER_DEFAULT_FPS, 1,
                                         NULL);
    if (caps == NULL)
    {
        printf("Failed to create caps\n");
        FinalizeGStreamerPipeline();

        return false;
    }

#else
    // support 4 channel 8 bit data
    stream << "video/x-raw"
           << ", width=" << wndWidth_
           << ", height=" << wndHeight_
           << ", format=(string){RGBA}"
           << ", framerate=" << GSTREAMER_DEFAULT_FPS << "/1;";
    GstCaps * caps = gst_caps_from_string(stream.str().c_str());

    if (caps == NULL)
    {
        printf("Failed to create caps\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    caps = gst_caps_fixate(caps);
#endif

    gst_app_src_set_caps(appsrc, caps);
    gst_caps_unref(caps);

    gst_bin_add(GST_BIN(pipeline), appsrcelem);

    // create color convert element
    GstElement * color = gst_element_factory_make(COLOR_ELEM, NULL);
    if (color == NULL)
    {
        printf("Cannot create " COLOR_ELEM " element\n");
        FinalizeGStreamerPipeline();

        return false;
    }
    gst_bin_add(GST_BIN(pipeline), color);

    // create videoflip element
    GstElement * videoflip = gst_element_factory_make("videoflip", NULL);
    if (videoflip == NULL)
    {
        printf("Cannot create videoflip element\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    g_object_set(G_OBJECT(videoflip), "method", 5, NULL);

    gst_bin_add(GST_BIN(pipeline), videoflip);

    // create encodelem element
    GstElement * encodelem = gst_element_factory_make(ENCODE_ELEM, NULL);
    if (encodelem == NULL)
    {
        printf("Cannot create " ENCODE_ELEM " element\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), encodelem);

    // create avimux element
    GstElement * avimux = gst_element_factory_make("avimux", NULL);
    if (avimux == NULL)
    {
        printf("Cannot create avimux element\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), avimux);

    // create filesink element
    GstElement * filesink = gst_element_factory_make("filesink", NULL);
    if (filesink == NULL)
    {
        printf("Cannot create filesink element\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    g_object_set(G_OBJECT(filesink), "location", windowTitle_.c_str(), NULL);
    g_object_set(G_OBJECT(filesink), "append", 0, NULL);

    gst_bin_add(GST_BIN(pipeline), filesink);


    // link elements
    if (!gst_element_link_many(appsrcelem, color, videoflip,
                               encodelem, avimux, filesink, NULL))
    {
        printf("GStreamer: cannot link appsrc -> " COLOR_ELEM
               " -> videoflip -> " ENCODE_ELEM " -> avimux -> filesink\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    // Force pipeline to play video as fast as possible, ignoring system clock
    gst_pipeline_use_clock(pipeline, NULL);

    num_frames = 0;

    GstStateChangeReturn status = gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
    if (status == GST_STATE_CHANGE_FAILURE)
    {
        printf("GStreamer: unable to start playback\n");
        FinalizeGStreamerPipeline();

        return false;
    }

    return true;
}

#endif // USE_GUI && USE_GSTREAMER
