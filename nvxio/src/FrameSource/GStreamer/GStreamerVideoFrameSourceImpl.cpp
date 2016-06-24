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

#if (defined(USE_GSTREAMER) && !defined(USE_HARDWARE_CODEC))

#include <memory>

#include "GStreamerVideoFrameSourceImpl.hpp"

#include <gst/pbutils/missing-plugins.h>
#include <gst/app/gstappsink.h>

#include <cuda_runtime_api.h>

#include <map>

namespace nvxio
{

GStreamerVideoFrameSourceImpl::GStreamerVideoFrameSourceImpl(vx_context context, const std::string & path):
    GStreamerBaseFrameSourceImpl(context, FrameSource::VIDEO_SOURCE, "GstreamerVideoFrameSource"),
    fileName(path)
{
#ifndef USE_GSTREAMER_OMX
    const std::map<std::string, guint> features_list =
    {
        { "omxmpeg2videodec", GST_RANK_NONE },
        { "omxvp9dec", GST_RANK_NONE },
        { "omxvp8dec", GST_RANK_NONE },
        { "omxh265dec", GST_RANK_NONE },
        { "omxh264dec", GST_RANK_NONE },
        { "omxmpeg4videodec", GST_RANK_NONE },
    };

    GstElementFactory* factory = NULL;
    for (auto p : features_list)
    {
        factory = gst_element_factory_find (p.first.c_str());
        if (factory)
        {
            gst_plugin_feature_set_rank (GST_PLUGIN_FEATURE (factory), p.second);
            gst_object_unref (factory);
        }
    }
#endif
}

GstAutoplugSelectResult GStreamerVideoFrameSourceImpl::autoPlugSelect(GstElement *, GstPad *,
                              GstCaps * caps, GstElementFactory *, gpointer)
{
    std::unique_ptr<char[], GlibDeleter> capsStr(gst_caps_to_string(caps));
    if (strstr(capsStr.get(), "video"))
    {
        return GST_AUTOPLUG_SELECT_TRY;
    }
    else
    {
        return GST_AUTOPLUG_SELECT_EXPOSE;
    }
}

bool GStreamerVideoFrameSourceImpl::InitializeGstPipeLine()
{
    GstStateChangeReturn status;
    end = true;

    std::string uri;
    if (!gst_uri_is_valid(fileName.c_str()))
    {
        char* real = realpath(fileName.c_str(), NULL);

        if (!real)
        {
            printf("Can't resolve path \"%s\": %s\n", fileName.c_str(), strerror(errno));
            return false;
        }

        std::unique_ptr<char[], GlibDeleter> pUri(g_filename_to_uri(real, NULL, NULL));
        free(real);
        uri = pUri.get();
    }
    else
    {
        uri = fileName;
    }

    pipeline = GST_PIPELINE(gst_pipeline_new(NULL));
    if (pipeline == NULL)
    {
        printf("Cannot create Gstreamer pipeline\n");
        return false;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE (pipeline));

    // create uridecodebin
    GstBin * uriDecodeBin = GST_BIN(gst_element_factory_make("uridecodebin", NULL));
    if (uriDecodeBin == NULL)
    {
        printf("Cannot create uridecodebin\n");
        FinalizeGstPipeLine();

        return false;
    }

    g_object_set(G_OBJECT(uriDecodeBin), "uri", uri.c_str(), NULL);
    gst_bin_add(GST_BIN(pipeline), GST_ELEMENT(uriDecodeBin));

    // create color convert
    GstElement * color = gst_element_factory_make(COLOR_ELEM, NULL);
    if (color == NULL)
    {
        printf("Cannot create %s element\n", COLOR_ELEM);
        FinalizeGstPipeLine();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), color);

    // create appsink
    sink = gst_element_factory_make("appsink", NULL);
    if (sink == NULL)
    {
        printf("Cannot create appsink element\n");
        FinalizeGstPipeLine();

        return false;
    }

    gst_bin_add(GST_BIN(pipeline), sink);

    g_signal_connect(uriDecodeBin, "autoplug-select", G_CALLBACK(GStreamerVideoFrameSourceImpl::autoPlugSelect), NULL);
    g_signal_connect(uriDecodeBin, "pad-added", G_CALLBACK(GStreamerBaseFrameSourceImpl::newGstreamerPad), color);

    // link elements
    if (!gst_element_link(color, sink))
    {
        printf("GStreamer: cannot link color -> sink\n");
        FinalizeGstPipeLine();

        return false;
    }

    gst_app_sink_set_max_buffers (GST_APP_SINK(sink), 4);
    gst_app_sink_set_drop (GST_APP_SINK(sink), false);
    gst_app_sink_set_emit_signals (GST_APP_SINK(sink), 0);

#if GST_VERSION_MAJOR == 0
    GstCaps* caps = gst_caps_new_simple("video/x-raw-rgb",
                               "bpp",        G_TYPE_INT, 24,
                               "red_mask",   G_TYPE_INT, 0xFF0000,
                               "green_mask", G_TYPE_INT, 0x00FF00,
                               "blue_mask",  G_TYPE_INT, 0x0000FF,
                               NULL);
#else
    // support 1 and 3 channel 8 bit data
    GstCaps* caps = gst_caps_from_string("video/x-raw, format=(string){RGB, GRAY8};");
#endif
    gst_app_sink_set_caps(GST_APP_SINK(sink), caps);
    gst_caps_unref(caps);

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

    //GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    std::unique_ptr<GstPad, GStreamerObjectDeleter> pad(gst_element_get_static_pad(color, "src"));

#if GST_VERSION_MAJOR == 0
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> bufferCaps(gst_pad_get_caps(pad.get()));
#else
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> bufferCaps(gst_pad_get_current_caps(pad.get()));
#endif

    if (!bufferCaps)
    {
        printf("Width, height, fps can not be queried\n");
#if FULL_GST_VERSION == VERSION_NUM(1, 4, 5)
        printf("Note: probably, %s is a video file with variable bit-rate, "
               "which can not be played due to GStreamer issue\n", fileName.c_str());
#endif
        FinalizeGstPipeLine();
        return false;
    }

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

    if (configuration.frameWidth == (vx_uint32)-1)
        configuration.frameWidth = width;
    if (configuration.frameHeight == (vx_uint32)-1)
        configuration.frameHeight = height;

    gint num = 0, denom = 1;
    if (!gst_structure_get_fraction(structure, "framerate", &num, &denom))
    {
        handleGStreamerMessages();
        printf("Cannot query video fps\n");
    }
    configuration.fps = (float)num/(float)denom;

    end = false;

    return true;
}

}

#endif
