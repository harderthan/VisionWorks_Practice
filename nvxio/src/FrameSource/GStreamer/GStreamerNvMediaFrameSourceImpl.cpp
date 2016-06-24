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

#ifdef USE_HARDWARE_CODEC

#include "NVXIO/Application.hpp"
#include "FrameSource/GStreamer/GStreamerNvMediaFrameSourceImpl.hpp"

#include <VX/vx.h>
#include <NVX/nvx.h>
#include <cuda_runtime.h>

#include <EGL/eglext.h>

namespace nvxio
{

GStreamerNvMediaFrameSourceImpl::GStreamerNvMediaFrameSourceImpl(vx_context vxcontext, const std::string & path):
    GStreamerEGLStreamSinkFrameSourceImpl(vxcontext, FrameSource::VIDEO_SOURCE, "GstreamerNvMediaFrameSource", true),
    fileName(path)
{
}

void GStreamerNvMediaFrameSourceImpl::setNvMediaPluginRunk()
{
    const std::map<std::string, guint> features_list =
    {
        { "avdec_mpeg4",          GST_RANK_SECONDARY },
        { "avdec_h264",           GST_RANK_SECONDARY },
        { "nvmediamp3auddec",     GST_RANK_PRIMARY },
        { "nvmediaaacauddec",     GST_RANK_PRIMARY },
        { "nvmediawmaauddec",     GST_RANK_PRIMARY },
        { "nvmediaaacaudenc",     GST_RANK_PRIMARY },
        { "nvmediampeg2viddec",   GST_RANK_PRIMARY },
        { "nvmediampeg4viddec",   GST_RANK_PRIMARY },
        { "nvmediavc1viddec",     GST_RANK_PRIMARY },
        { "nvmediamjpegviddec",   GST_RANK_PRIMARY },
        { "nvmediah264viddec",    GST_RANK_PRIMARY },
        { "nvmediah264videnc",    GST_RANK_PRIMARY },
        { "nvmediacapturesrc",    GST_RANK_PRIMARY },
        { "nvmediaoverlaysink",   GST_RANK_PRIMARY },
        { "nvmediaeglstreamsink", GST_RANK_PRIMARY },
        { "nvmediavp8viddec",     GST_RANK_PRIMARY }
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
}
void GStreamerNvMediaFrameSourceImpl::newGstreamerPad(GstElement * /*elem*/, GstPad *pad, gpointer data)
{
    GstPad *sinkpad;
    GstElement *mixer = (GstElement *) data;

    sinkpad = gst_element_get_static_pad (mixer, "sink");
    if (!sinkpad)
    {
        printf("Gstreamer: no pad named \"sink\"\n");
        return;
    }

    gst_pad_link(pad, sinkpad);
    gst_object_unref(sinkpad);
}

GStreamerNvMediaFrameSourceImpl::GstAutoplugSelectResult GStreamerNvMediaFrameSourceImpl::autoPlugSelect(GstElement *, GstPad *,
                              GstCaps * caps, GstElementFactory *, gpointer)
{
    gchar * msg_str = gst_caps_to_string(caps);
    GStreamerNvMediaFrameSourceImpl::GstAutoplugSelectResult result = GST_AUTOPLUG_SELECT_EXPOSE;

    if (strstr(msg_str, "video"))
    {
        result = GST_AUTOPLUG_SELECT_TRY;
    }

    g_free(msg_str);

    return result;
}

bool GStreamerNvMediaFrameSourceImpl::InitializeGstPipeLine()
{
    end = true;
    setNvMediaPluginRunk();

    GstStateChangeReturn status;

    std::string uri;
    const char * fileNameS = fileName.c_str();

    if(!gst_uri_is_valid(fileNameS))
    {
        char* real = realpath(fileNameS, NULL);
        if (!real)
        {
            printf("Can't resolve path \"%s\": %s\n", fileNameS, strerror(errno));
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

    // create uriDecodeBin
    GstElement * uriDecodeBin = gst_element_factory_make("uridecodebin", NULL);
    handleGStreamerMessages();
    if (uriDecodeBin == NULL)
    {
        printf("Cannot create uridecodebin\n");
        return false;
    }

    g_object_set(G_OBJECT(uriDecodeBin), "uri", uri.c_str(), NULL);
    GstCaps* caps = gst_caps_from_string("video/x-nvmedia");
    g_object_set(G_OBJECT(uriDecodeBin), "caps", caps, NULL);
    gst_caps_unref(caps);

    gst_bin_add(GST_BIN(pipeline), uriDecodeBin);

    // create nvmediasurfmixer
    GstElement * surfaceMixer = gst_element_factory_make("nvmediasurfmixer", NULL);
    if (surfaceMixer == NULL)
    {
        printf("Cannot create surface mixer\n");
        FinalizeGstPipeLine();
        return false;
    }

    gst_bin_add(GST_BIN(pipeline), surfaceMixer);

    g_signal_connect(uriDecodeBin, "pad-added", G_CALLBACK(GStreamerNvMediaFrameSourceImpl::newGstreamerPad), surfaceMixer);
    g_signal_connect(uriDecodeBin, "autoplug-select", G_CALLBACK(GStreamerNvMediaFrameSourceImpl::autoPlugSelect), NULL);

    // create nvmediaeglstreamsink
    GstElement * eglSink = gst_element_factory_make("nvmediaeglstreamsink", NULL);
    if (eglSink == NULL)
    {
        printf("Cannot create EGL sink\n");
        FinalizeGstPipeLine();
        return false;
    }

    gst_bin_add(GST_BIN(pipeline), eglSink);

    g_object_set(G_OBJECT(eglSink), "display", context.display, NULL);
    g_object_set(G_OBJECT(eglSink), "stream", context.stream, NULL);
    g_object_set(G_OBJECT(eglSink), "fifo", fifoMode, NULL);
    g_object_set(G_OBJECT(eglSink), "max-lateness", -1, NULL);
    g_object_set(G_OBJECT(eglSink), "throttle-time", 0, NULL);
    g_object_set(G_OBJECT(eglSink), "render-delay", 0, NULL);
    g_object_set(G_OBJECT(eglSink), "qos", FALSE, NULL);
    g_object_set(G_OBJECT(eglSink), "sync", FALSE, NULL);
    g_object_set(G_OBJECT(eglSink), "async", TRUE, NULL);

    // link elements
    if (!gst_element_link(surfaceMixer, eglSink))
    {
        printf("Cannot link SurfaceMixer and EGL sink\n");
        FinalizeGstPipeLine();
        return false;
    }

    // Force pipeline to play video as fast as possible, ignoring system clock
    gst_pipeline_use_clock(pipeline, NULL);

    status = gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
    handleGStreamerMessages();

    if (status == GST_STATE_CHANGE_ASYNC)
    {
        // wait for status update
        status = gst_element_get_state(GST_ELEMENT(pipeline), NULL, NULL, GST_CLOCK_TIME_NONE);
    }

//    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    if (status == GST_STATE_CHANGE_FAILURE)
    {
        printf("GStreamer: unable to start playback\n");
        FinalizeGstPipeLine();
        return false;
    }

    GstPad* pad = gst_element_get_static_pad(surfaceMixer, "src");
    GstCaps* buffer_caps = gst_pad_get_current_caps(pad);
    const GstStructure *structure = gst_caps_get_structure (buffer_caps, 0);

    int width, height;
    if (!gst_structure_get_int(structure, "width", &width))
    {
        handleGStreamerMessages();
        printf("Cannot query video width\n");
    }

    if (!gst_structure_get_int (structure, "height", &height))
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
    configuration.fps = num / static_cast<float>(denom);

    end = false;

    return true;
}

}

#endif
