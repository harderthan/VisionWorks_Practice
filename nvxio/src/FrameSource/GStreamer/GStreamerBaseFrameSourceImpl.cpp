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

#ifdef USE_GSTREAMER

#include <memory>

#include "NVXIO/Application.hpp"
#include "NVXIO/Utility.hpp"
#include "FrameSource/GStreamer/GStreamerBaseFrameSourceImpl.hpp"

#include <gst/pbutils/missing-plugins.h>
#include <gst/app/gstappsink.h>

#include <cuda_runtime_api.h>

namespace nvxio
{

GStreamerBaseFrameSourceImpl::GStreamerBaseFrameSourceImpl(vx_context context, FrameSource::SourceType type, const std::string & name):
    FrameSource(type, name),
    pipeline(NULL), bus(NULL),
    end(true),
    vxContext(context),
    sink(NULL),
    devMem(NULL),
    devMemPitch(0),
    scaledImage(NULL)
{
}

void GStreamerBaseFrameSourceImpl::newGstreamerPad(GstElement * /*elem*/, GstPad *pad, gpointer data)
{
    GstElement *color = (GstElement *) data;

    GstPad* sinkpad = gst_element_get_static_pad (color, "sink");
    if (!sinkpad)
    {
        //printf("Gstreamer: no pad named sink\n");
        return;
    }

    gst_pad_link(pad, sinkpad);
    gst_object_unref(sinkpad);
}

bool GStreamerBaseFrameSourceImpl::open()
{
    if (pipeline)
    {
        close();
    }

    if (!InitializeGstPipeLine())
    {
        printf("Cannot initialize Gstreamer pipeline\n");
        return false;
    }

    lastFrameTimestamp.tic();

    NVXIO_ASSERT(!end);

    return true;
}

FrameSource::FrameStatus GStreamerBaseFrameSourceImpl::fetch(vx_image image, vx_uint32 /*timeout*/)
{
    if (end)
    {
        close();
        return FrameSource::CLOSED;
    }

    handleGStreamerMessages();

    if (gst_app_sink_is_eos(GST_APP_SINK(sink)))
    {
        close();
        return FrameSource::CLOSED;
    }

    if ((lastFrameTimestamp.toc()/1000.0) > Application::get().getSourceDefaultTimeout())
    {
        close();
        return FrameSource::CLOSED;
    }

    lastFrameTimestamp.tic();

#if GST_VERSION_MAJOR == 0
    std::unique_ptr<GstBuffer, GStreamerObjectDeleter> bufferHolder(
        gst_app_sink_pull_buffer(GST_APP_SINK(sink)));
    GstBuffer* buffer = bufferHolder.get();
#else
    std::unique_ptr<GstSample, GStreamerObjectDeleter> sample(gst_app_sink_pull_sample(GST_APP_SINK(sink)));

    if (!sample)
    {
        close();
        return FrameSource::CLOSED;
    }

    GstBuffer* buffer = gst_sample_get_buffer(sample.get());
#endif

    gint          width;
    gint          height;

#if GST_VERSION_MAJOR == 0
    std::unique_ptr<GstCaps, GStreamerObjectDeleter> bufferCapsHolder(gst_buffer_get_caps(buffer));
    GstCaps* bufferCaps = bufferCapsHolder.get();
#else
    GstCaps* bufferCaps = gst_sample_get_caps(sample.get());
#endif
    // bail out in no caps
    assert(gst_caps_get_size(bufferCaps) == 1);
    GstStructure* structure = gst_caps_get_structure(bufferCaps, 0);

    // bail out if width or height are 0
    if (!gst_structure_get_int(structure, "width", &width) ||
            !gst_structure_get_int(structure, "height", &height))
    {
        close();
        return FrameSource::CLOSED;
    }

    int depth = 3;
#if GST_VERSION_MAJOR > 0
    depth = 0;
    const gchar* name = gst_structure_get_name(structure);
    const gchar* format = gst_structure_get_string(structure, "format");

    if (!name || !format)
    {
        close();
        return FrameSource::CLOSED;
    }

    // we support 2 types of data:
    //     video/x-raw, format=BGR   -> 8bit, 3 channels
    //     video/x-raw, format=GRAY8 -> 8bit, 1 channel
    if (strcasecmp(name, "video/x-raw") == 0)
    {
        if (strcasecmp(format, "RGB") == 0)
        {
            depth = 3;
        }
        else if(strcasecmp(format, "GRAY8") == 0)
        {
            depth = 1;
        }
    }
#endif
    if (depth == 0)
    {
        close();
        return FrameSource::CLOSED;
    }

    vx_imagepatch_addressing_t decodedImageAddr;
    decodedImageAddr.dim_x = width;
    decodedImageAddr.dim_y = height;
    decodedImageAddr.stride_x = depth;
    // GStreamer uses as stride width rounded up to the nearest multiple of 4
    decodedImageAddr.stride_y = ((width*depth+3)/4)*4;
    decodedImageAddr.scale_x = 1;
    decodedImageAddr.scale_y = 1;
    vx_image decodedImage = NULL;
    vx_df_image_e vx_type_map[5] = { VX_DF_IMAGE_VIRT, VX_DF_IMAGE_U8,
                                     VX_DF_IMAGE_VIRT, VX_DF_IMAGE_RGB, VX_DF_IMAGE_RGBX };

    // fetch image width and height
    vx_uint32 actual_width, actual_height;
    vx_df_image_e actual_format;
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, (void *)&actual_width, sizeof(actual_width)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, (void *)&actual_height, sizeof(actual_height)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, (void *)&actual_format, sizeof(actual_format)) );
    bool needScale = width != (int)configuration.frameWidth || height != (int)configuration.frameHeight;

    // config and actual image sized must be the same!
    if ((actual_height != configuration.frameHeight) ||
            (actual_width != configuration.frameWidth) ||
            (actual_format != configuration.format))
    {
        close();

        NVXIO_THROW_EXCEPTION("Actual image [ " << actual_width << " x " << actual_height <<
                              " ] does not equal configuration one [ " << configuration.frameWidth
                              << " x " << configuration.frameHeight << " ]");
    }

    // we assume that decoced image will have no more than 3 channels per pixel
    if (!devMem)
    {
        NVXIO_ASSERT( cudaSuccess == cudaMallocPitch(&devMem, &devMemPitch, width * 3, height) );
    }

    // check if decoded image format has changed
    if (scaledImage)
    {
        vx_df_image_e scaled_format;
        NVXIO_SAFE_CALL( vxQueryImage(scaledImage, VX_IMAGE_ATTRIBUTE_FORMAT, (void *)&scaled_format, sizeof(scaled_format)) );

        if (scaled_format != vx_type_map[depth])
        {
            vxReleaseImage(&scaledImage);
            scaledImage = NULL;
        }
    }

    if (needScale && !scaledImage)
    {
        scaledImage = vxCreateImage(vxContext, configuration.frameWidth,
                                    configuration.frameHeight, vx_type_map[depth]);
        NVXIO_CHECK_REFERENCE( scaledImage );
    }

#if GST_VERSION_MAJOR == 0
    bool needConvert = configuration.format != VX_DF_IMAGE_RGB;
    void * decodedPtr = GST_BUFFER_DATA(buffer);
#else
    GstMapInfo info;

    gboolean success = gst_buffer_map(buffer, &info, (GstMapFlags)GST_MAP_READ);
    if (!success)
    {
        printf("GStreamer: unable to map buffer\n");
        close();
        return FrameSource::CLOSED;
    }

    bool needConvert = configuration.format != vx_type_map[depth];
    void * decodedPtr = info.data;
#endif

    if (!needConvert && !needScale)
    {
        decodedImage = vxCreateImageFromHandle(vxContext, vx_type_map[depth], &decodedImageAddr,
                                               &decodedPtr, VX_IMPORT_TYPE_HOST);
        NVXIO_CHECK_REFERENCE( decodedImage );
        NVXIO_SAFE_CALL( nvxuCopyImage(vxContext, decodedImage, image) );
    }
    else
    {
        // 1. upload decoced image to CUDA buffer
        NVXIO_ASSERT( cudaSuccess == cudaMemcpy2D(devMem, devMemPitch,
                                                  decodedPtr, decodedImageAddr.stride_y,
                                                  decodedImageAddr.dim_x * depth, decodedImageAddr.dim_y,
                                                  cudaMemcpyHostToDevice) );

        // 2. create vx_image wrapper for decoded buffer
        decodedImageAddr.stride_y = static_cast<vx_int32>(devMemPitch);
        decodedImage = vxCreateImageFromHandle(vxContext, vx_type_map[depth], &decodedImageAddr,
                                               &devMem, NVX_IMPORT_TYPE_CUDA);
        NVXIO_CHECK_REFERENCE( decodedImage );

        if (needScale)
        {
            // 3. scale image
            NVXIO_SAFE_CALL( vxuScaleImage(vxContext, decodedImage, scaledImage, VX_INTERPOLATION_TYPE_BILINEAR) );

            // 4. convert to dst image
            NVXIO_SAFE_CALL( vxuColorConvert(vxContext, scaledImage, image) );
        }
        else
        {
            // 3. convert to dst image
            NVXIO_SAFE_CALL( vxuColorConvert(vxContext, decodedImage, image) );
        }
    }

#if GST_VERSION_MAJOR != 0
    gst_buffer_unmap(buffer, &info);
#endif

    NVXIO_SAFE_CALL( vxReleaseImage(&decodedImage) );

    return FrameSource::OK;
}

FrameSource::Parameters GStreamerBaseFrameSourceImpl::getConfiguration()
{
    return configuration;
}

bool GStreamerBaseFrameSourceImpl::setConfiguration(const FrameSource::Parameters& params)
{
    bool result = true;

    if (end)
    {
        configuration.frameHeight = params.frameHeight;
        configuration.frameWidth = params.frameWidth;
    }
    else
    {
        if ((params.frameWidth != (vx_uint32)-1) && (params.frameWidth != configuration.frameWidth))
            result = false;
        if ((params.frameHeight != (vx_uint32)-1) && (params.frameHeight != configuration.frameHeight))
            result = false;
    }

    if ((params.fps != (vx_uint32)-1) && (params.fps != configuration.fps))
        result = false;

    configuration.format = params.format;

    return result;
}

void GStreamerBaseFrameSourceImpl::close()
{
    handleGStreamerMessages();
    FinalizeGstPipeLine();

    if (devMem != NULL)
    {
        cudaFree(devMem);
        devMem = NULL;
    }

    if (scaledImage)
    {
        vxReleaseImage(&scaledImage);
        scaledImage = NULL;
    }
}

void GStreamerBaseFrameSourceImpl::handleGStreamerMessages()
{
    GstMessage* msg  = NULL;
    GError *err = NULL;
    gchar *debug = NULL;
    GstStreamStatusType tp;
    GstElement * elem = NULL;

    if (!bus)
        return;

    while (gst_bus_have_pending(bus))
    {
        msg = gst_bus_pop(bus);

        if (gst_is_missing_plugin_message(msg))
        {
            printf("GStreamer: your gstreamer installation is missing a required plugin!\n");
            end = true;
        }
        else
        {
            switch (GST_MESSAGE_TYPE(msg))
            {
                case GST_MESSAGE_STATE_CHANGED:
                    GstState oldstate, newstate, pendstate;
                    gst_message_parse_state_changed(msg, &oldstate, &newstate, &pendstate);
                    break;
                case GST_MESSAGE_ERROR:
                {
                    gst_message_parse_error(msg, &err, &debug);
                    std::unique_ptr<char[], GlibDeleter> name(gst_element_get_name(GST_MESSAGE_SRC (msg)));

                    printf("GStreamer Plugin: Embedded video playback halted; module %s reported: %s\n",
                           name.get(), err->message);

                    g_error_free(err);
                    g_free(debug);
                    end = true;
                    break;
                }
                case GST_MESSAGE_EOS:
                    end = true;
                    break;
                case GST_MESSAGE_STREAM_STATUS:
                    gst_message_parse_stream_status(msg,&tp,&elem);
                    break;
                default:
                    break;
            }
        }

        gst_message_unref(msg);
    }
}

void GStreamerBaseFrameSourceImpl::FinalizeGstPipeLine()
{
    if (pipeline)
    {
        handleGStreamerMessages();

        gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
        handleGStreamerMessages();

        gst_object_unref(GST_OBJECT(bus));
        bus = NULL;

        gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
        gst_object_unref(GST_OBJECT(pipeline));
        pipeline = NULL;
    }
}

GStreamerBaseFrameSourceImpl::~GStreamerBaseFrameSourceImpl()
{
    close();
}

}

#endif
