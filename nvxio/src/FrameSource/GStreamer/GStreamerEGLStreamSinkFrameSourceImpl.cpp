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

#if defined USE_HARDWARE_CODEC || defined USE_NVGSTCAMERA

#include "NVXIO/FrameSource.hpp"
#include "NVXIO/Application.hpp"
#include "FrameSource/GStreamer/GStreamerEGLStreamSinkFrameSourceImpl.hpp"

#include <VX/vx.h>
#include <NVX/nvx.h>
#include <cuda_runtime.h>

#include <gst/pbutils/missing-plugins.h>

#include <EGL/egl.h>
#include <EGL/eglext.h>

#include <memory>
#include <thread>
#include <string>

#define EXTLST_DECL(tx, x) tx x = NULL;
EXTENSION_LIST(EXTLST_DECL)

typedef void (* extlst_fnptr_t)(void);
#define EXTLST_ENTRY(tx, x) { (extlst_fnptr_t *)&x, #x },

static struct
{
    extlst_fnptr_t * fnptr;
    char const * name;
} extensionList[] = { EXTENSION_LIST(EXTLST_ENTRY) };

namespace nvxio
{

GStreamerEGLStreamSinkFrameSourceImpl::GStreamerEGLStreamSinkFrameSourceImpl(vx_context vxcontext, SourceType sourceType,
                                                                             const char * const name, bool fifomode) :
    FrameSource(sourceType, name),
    pipeline(NULL),
    bus(NULL),
    end(true),
    fifoLength(4),
    fifoMode(fifomode),
    latency(0),
    cudaConnection(NULL),
    vxContext(vxcontext),
    scaledImage(NULL)
{
    context.stream = EGL_NO_STREAM_KHR;
    context.display = EGL_NO_DISPLAY;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::open()
{
    if (pipeline)
    {
        close();
    }

    printf("Initializing EGL display\n");
    if (!InitializeEGLDisplay())
    {
        printf("Cannot initialize EGL display\n");
        return false;
    }

    printf("Initializing EGL stream\n");
    if (!InitializeEGLStream())
    {
        printf("Cannot initialize EGL Stream\n");
        return false;
    }

    printf("Initializing EGL consumer\n");
    if (!InitializeEglCudaConsumer())
    {
        printf("Cannot initialize CUDA consumer\n");
        return false;
    }

    printf("Creating GStreamer pipeline\n");
    if (!InitializeGstPipeLine())
    {
        printf("Cannot initialize Gstreamer pipeline\n");
        return false;
    }

    lastFrameTimestamp.tic();

    return true;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::InitializeEGLDisplay()
{
    // Obtain the EGL display
    context.display = nvxio::EGLDisplayAccessor::getInstance();
    if (context.display == EGL_NO_DISPLAY)
    {
        printf("EGL failed to obtain display.\n");
        return false;
    }

    return true;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::InitializeEglCudaConsumer()
{
    printf("Connect CUDA consumer\n");
    CUresult curesult = cuEGLStreamConsumerConnect(&cudaConnection, context.stream);
    if (CUDA_SUCCESS != curesult)
    {
        printf("Connect CUDA consumer ERROR %d\n", curesult);
        return false;
    }

    return true;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::setupExtensions()
{
    for (size_t i = 0; i < (sizeof(extensionList) / sizeof(*extensionList)); i++)
    {
        *extensionList[i].fnptr = eglGetProcAddress(extensionList[i].name);
        if (*extensionList[i].fnptr == NULL)
        {
            printf("Couldn't get address of %s()\n", extensionList[i].name);
            return false;
        }
    }

    return true;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::InitializeEGLStream()
{
    const EGLint streamAttrMailboxMode[] = { EGL_NONE };
    const EGLint streamAttrFIFOMode[] = { EGL_STREAM_FIFO_LENGTH_KHR, fifoLength, EGL_NONE };

    if(!setupExtensions())
        return false;

    context.stream = eglCreateStreamKHR(context.display, fifoMode ? streamAttrFIFOMode : streamAttrMailboxMode);
    if (context.stream == EGL_NO_STREAM_KHR)
    {
        printf("Couldn't create stream.\n");
        return false;
    }

    if (!eglStreamAttribKHR(context.display, context.stream, EGL_CONSUMER_LATENCY_USEC_KHR, latency))
    {
        printf("Consumer: streamAttribKHR EGL_CONSUMER_LATENCY_USEC_KHR failed\n");
    }
    if (!eglStreamAttribKHR(context.display, context.stream, EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR, 0))
    {
        printf("Consumer: streamAttribKHR EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR failed\n");
    }

    // Get stream attributes
    if (!eglQueryStreamKHR(context.display, context.stream, EGL_STREAM_FIFO_LENGTH_KHR, &fifoLength))
    {
        printf("Consumer: eglQueryStreamKHR EGL_STREAM_FIFO_LENGTH_KHR failed\n");
    }
    if (!eglQueryStreamKHR(context.display, context.stream, EGL_CONSUMER_LATENCY_USEC_KHR, &latency))
    {
        printf("Consumer: eglQueryStreamKHR EGL_CONSUMER_LATENCY_USEC_KHR failed\n");
    }

    if (fifoMode != (fifoLength > 0))
    {
        printf("EGL Stream consumer - Unable to set FIFO mode\n");
        fifoMode = false;
    }
    if (fifoMode)
    {
        printf("EGL Stream consumer - Mode: FIFO Length: %d\n",  fifoLength);
    }
    else
    {
        printf("EGL Stream consumer - Mode: Mailbox\n");
    }

    return true;
}

FrameSource::FrameStatus GStreamerEGLStreamSinkFrameSourceImpl::fetch(vx_image image, vx_uint32 timeout)
{
    handleGStreamerMessages();

    if (end)
    {
        close();
        return FrameSource::CLOSED;
    }

    CUgraphicsResource cudaResource;
    CUeglFrame eglFrame;
    EGLint streamState = 0;

    if (!eglQueryStreamKHR(context.display, context.stream, EGL_STREAM_STATE_KHR, &streamState))
    {
        printf("Cuda consumer, eglQueryStreamKHR EGL_STREAM_STATE_KHR failed\n");
        close();
        return FrameSource::CLOSED;
    }

    if (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)
    {
        printf("CUDA Consumer: - EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
        close();
        return FrameSource::CLOSED;
    }

    if (streamState != EGL_STREAM_STATE_NEW_FRAME_AVAILABLE_KHR)
    {
        if ((lastFrameTimestamp.toc()/1000.0) > Application::get().getSourceDefaultTimeout())
        {
            close();
            return FrameSource::CLOSED;
        }
        else
        {
            return FrameSource::TIMEOUT;
        }
    }
    else
    {
        lastFrameTimestamp.tic();
    }

    CUresult cuStatus = cuEGLStreamConsumerAcquireFrame(&cudaConnection, &cudaResource, NULL, timeout*1000);
    if (cuStatus != CUDA_SUCCESS)
    {
        printf("Cuda Acquire failed cuStatus=%d\n", cuStatus);
        close();
        return FrameSource::CLOSED;
    }

    cuStatus = cuGraphicsResourceGetMappedEglFrame(&eglFrame, cudaResource, 0, 0);
    if (cuStatus != CUDA_SUCCESS)
    {
        printf("Cuda get resource failed with %d\n", cuStatus);
        cuEGLStreamConsumerReleaseFrame(&cudaConnection, cudaResource, NULL);
        close();
        return FrameSource::CLOSED;
    }

    if ((eglFrame.width != configuration.frameWidth) || (eglFrame.height != configuration.frameHeight))
    {
        printf("Unexpected frame geometry change\n");
        cuEGLStreamConsumerReleaseFrame(&cudaConnection, cudaResource, NULL);
        close();
        return FrameSource::CLOSED;
    }

    vx_imagepatch_addressing_t srcImageAddr;
    srcImageAddr.dim_x = eglFrame.width;
    srcImageAddr.dim_y = eglFrame.height;
    srcImageAddr.stride_x = 4;
    srcImageAddr.stride_y = eglFrame.pitch;
    srcImageAddr.scale_x = srcImageAddr.scale_y = VX_SCALE_UNITY;
    srcImageAddr.step_x = srcImageAddr.step_y = 1;

    void *ptrs[] = { eglFrame.frame.pPitch[0] };
    vx_image srcImage = vxCreateImageFromHandle(vxContext, VX_DF_IMAGE_RGBX, &srcImageAddr,
                                                ptrs, NVX_IMPORT_TYPE_CUDA);
    NVXIO_CHECK_REFERENCE( srcImage );

    // fetch image width and height
    vx_uint32 actual_width, actual_height;
    vx_df_image_e actual_format;
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, (void *)&actual_width, sizeof(actual_width)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, (void *)&actual_height, sizeof(actual_height)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, (void *)&actual_format, sizeof(actual_format)) );
    bool needScale = eglFrame.width != configuration.frameWidth || eglFrame.height != configuration.frameHeight;

    // config and actual image sized must be the same!
    if ((actual_height != configuration.frameHeight) ||
            (actual_width != configuration.frameWidth) ||
            (actual_format != configuration.format))
    {
        cuStatus = cuEGLStreamConsumerReleaseFrame(&cudaConnection, cudaResource, NULL);
        close();

        NVXIO_THROW_EXCEPTION("Actual image config [ " << actual_width << " x " << actual_height <<
                              " ] does not equal configuration one [ " << configuration.frameWidth
                              << " x " << configuration.frameHeight << " ]");
    }

    if (needScale && !scaledImage &&
            (configuration.format != VX_DF_IMAGE_RGBX))
    {
        scaledImage = vxCreateImage(vxContext, configuration.frameWidth,
                                    configuration.frameHeight, VX_DF_IMAGE_RGBX);
        NVXIO_CHECK_REFERENCE( scaledImage );
    }

    if (needScale)
    {
        NVXIO_SAFE_CALL( vxuScaleImage(vxContext, srcImage, scaledImage ? scaledImage : image,
                                       VX_INTERPOLATION_TYPE_BILINEAR) );
    }
    else
    {
        scaledImage = srcImage;
    }

    switch(configuration.format)
    {
    case VX_DF_IMAGE_RGBX:
        if (!needScale)
        {
            NVXIO_SAFE_CALL( nvxuCopyImage(vxContext, scaledImage, image) );
        }
        break;
    case VX_DF_IMAGE_RGB:
        NVXIO_SAFE_CALL( vxuColorConvert(vxContext, scaledImage, image) );
        break;
    case VX_DF_IMAGE_U8:
        NVXIO_SAFE_CALL( vxuColorConvert(vxContext, scaledImage, image) );
        break;
    }

    if (!needScale)
    {
        scaledImage = NULL;
    }

    NVXIO_SAFE_CALL( vxReleaseImage(&srcImage) );
    cuStatus = cuEGLStreamConsumerReleaseFrame(&cudaConnection, cudaResource, NULL);

    return FrameSource::OK;
}

FrameSource::Parameters GStreamerEGLStreamSinkFrameSourceImpl::getConfiguration()
{
    return configuration;
}

bool GStreamerEGLStreamSinkFrameSourceImpl::setConfiguration(const FrameSource::Parameters& params)
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

void GStreamerEGLStreamSinkFrameSourceImpl::handleGStreamerMessages()
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
                    gst_message_parse_state_changed(msg, NULL, NULL, NULL);
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
                    gst_message_parse_stream_status(msg, &tp, &elem);
                    break;
                default:
                    break;
            }
        }

        gst_message_unref(msg);
    }
}

void GStreamerEGLStreamSinkFrameSourceImpl::FinalizeEglStream()
{
    if (context.stream != EGL_NO_STREAM_KHR)
    {
        eglDestroyStreamKHR(context.display, context.stream);
        context.stream = EGL_NO_STREAM_KHR;
    }
}

void GStreamerEGLStreamSinkFrameSourceImpl::FinalizeEglCudaConsumer()
{
    if (cudaConnection != NULL)
    {
        cuEGLStreamConsumerDisconnect(&cudaConnection);
        cudaConnection = NULL;
    }
}

void GStreamerEGLStreamSinkFrameSourceImpl::CloseGstPipeLineAsyncThread()
{
    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);
    end = true;
}

void GStreamerEGLStreamSinkFrameSourceImpl::FinalizeGstPipeLine()
{
    if (pipeline != NULL)
    {
        std::thread t(&GStreamerEGLStreamSinkFrameSourceImpl::CloseGstPipeLineAsyncThread, this);
        
        if (fifoMode)
        {
            CUgraphicsResource cudaResource;
            EGLint streamState = 0;
            while (!end)
            {
                if (!eglQueryStreamKHR(context.display, context.stream, EGL_STREAM_STATE_KHR, &streamState))
                {
                    handleGStreamerMessages();
                    break;
                }

                if (streamState == EGL_STREAM_STATE_NEW_FRAME_AVAILABLE_KHR)
                {
                    cuEGLStreamConsumerAcquireFrame(&cudaConnection, &cudaResource, NULL, 1000);
                    cuEGLStreamConsumerReleaseFrame(&cudaConnection, cudaResource, NULL);
                }
                else
                {
                    handleGStreamerMessages();
                    continue;
                }
                handleGStreamerMessages();
            }

        }

        t.join();

        gst_object_unref(GST_OBJECT(bus));
        bus = NULL;

        gst_object_unref(GST_OBJECT(pipeline));
        pipeline = NULL;
    }
}

void GStreamerEGLStreamSinkFrameSourceImpl::close()
{
    handleGStreamerMessages();
    FinalizeGstPipeLine();
    FinalizeEglCudaConsumer();
    FinalizeEglStream();

    if (scaledImage)
    {
        vxReleaseImage(&scaledImage);
        scaledImage = NULL;
    }
}

GStreamerEGLStreamSinkFrameSourceImpl::~GStreamerEGLStreamSinkFrameSourceImpl()
{
    close();
}

}

#endif // defined USE_HARDWARE_CODEC || defined USE_NVGSTCAMERA
