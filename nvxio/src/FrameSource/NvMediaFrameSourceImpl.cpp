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

#ifdef USE_NVMEDIA

#include "NVXIO/FrameSource.hpp"
#include "NVXIO/Application.hpp"
#include "FrameSource/NvMediaFrameSourceImpl.hpp"

#define MAX_ATTRIB 31
#define READ_SIZE (32*1024)
#define COPYFIELD(a, b, field) (a)->field = (b)->field

#define EXTENSION_LIST(T) \
T( PFNEGLCREATESTREAMKHRPROC,          eglCreateStreamKHR ) \
T( PFNEGLDESTROYSTREAMKHRPROC,         eglDestroyStreamKHR ) \
T( PFNEGLQUERYSTREAMKHRPROC,           eglQueryStreamKHR ) \
T( PFNEGLSTREAMATTRIBKHRPROC,          eglStreamAttribKHR )

#define EXTLST_DECL(tx, x) static tx x = NULL;
#define EXTLST_ENTRY(tx, x) { (extlst_fnptr_t *)&x, #x },

EXTENSION_LIST(EXTLST_DECL)
typedef void (*extlst_fnptr_t)(void);
static struct
{
    extlst_fnptr_t *fnptr;
    char const *name;
} extensionList[] = { EXTENSION_LIST(EXTLST_ENTRY) };

static bool setupExtensions()
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

static void SetParamsH264(NVDPictureData *pd, NvMediaPictureInfo *pictureInfo)
{
    NVH264PictureData *h264Data = &pd->CodecSpecific.h264;
    NvMediaPictureInfoH264 *h264PictureInfo = (NvMediaPictureInfoH264 *)pictureInfo;

    h264PictureInfo->field_order_cnt[0] = h264Data->CurrFieldOrderCnt[0];
    h264PictureInfo->field_order_cnt[1] = h264Data->CurrFieldOrderCnt[1];
    h264PictureInfo->is_reference = pd->ref_pic_flag;
    h264PictureInfo->chroma_format_idc = pd->chroma_format;
    COPYFIELD(h264PictureInfo, h264Data, frame_num);
    COPYFIELD(h264PictureInfo, pd, field_pic_flag);
    COPYFIELD(h264PictureInfo, pd, bottom_field_flag);
    COPYFIELD(h264PictureInfo, h264Data, num_ref_frames);
    h264PictureInfo->mb_adaptive_frame_field_flag = h264Data->MbaffFrameFlag;
    COPYFIELD(h264PictureInfo, h264Data, constrained_intra_pred_flag);
    COPYFIELD(h264PictureInfo, h264Data, weighted_pred_flag);
    COPYFIELD(h264PictureInfo, h264Data, weighted_bipred_idc);
    COPYFIELD(h264PictureInfo, h264Data, frame_mbs_only_flag);
    COPYFIELD(h264PictureInfo, h264Data, transform_8x8_mode_flag);
    COPYFIELD(h264PictureInfo, h264Data, chroma_qp_index_offset);
    COPYFIELD(h264PictureInfo, h264Data, second_chroma_qp_index_offset);
    COPYFIELD(h264PictureInfo, h264Data, pic_init_qp_minus26);
    COPYFIELD(h264PictureInfo, h264Data, num_ref_idx_l0_active_minus1);
    COPYFIELD(h264PictureInfo, h264Data, num_ref_idx_l1_active_minus1);
    COPYFIELD(h264PictureInfo, h264Data, log2_max_frame_num_minus4);
    COPYFIELD(h264PictureInfo, h264Data, pic_order_cnt_type);
    COPYFIELD(h264PictureInfo, h264Data, log2_max_pic_order_cnt_lsb_minus4);
    COPYFIELD(h264PictureInfo, h264Data, delta_pic_order_always_zero_flag);
    COPYFIELD(h264PictureInfo, h264Data, direct_8x8_inference_flag);
    COPYFIELD(h264PictureInfo, h264Data, entropy_coding_mode_flag);
    COPYFIELD(h264PictureInfo, h264Data, pic_order_present_flag);
    COPYFIELD(h264PictureInfo, h264Data, deblocking_filter_control_present_flag);
    COPYFIELD(h264PictureInfo, h264Data, redundant_pic_cnt_present_flag);
    COPYFIELD(h264PictureInfo, h264Data, num_slice_groups_minus1);
    COPYFIELD(h264PictureInfo, h264Data, slice_group_map_type);
    COPYFIELD(h264PictureInfo, h264Data, slice_group_change_rate_minus1);
    h264PictureInfo->slice_group_map = h264Data->pMb2SliceGroupMap;
    COPYFIELD(h264PictureInfo, h264Data, fmo_aso_enable);
    COPYFIELD(h264PictureInfo, h264Data, scaling_matrix_present);

    memcpy(h264PictureInfo->scaling_lists_4x4, h264Data->WeightScale4x4, sizeof(h264Data->WeightScale4x4));
    memcpy(h264PictureInfo->scaling_lists_8x8, h264Data->WeightScale8x8, sizeof(h264Data->WeightScale8x8));

    // nvdec specific, not required for avp+vde
    COPYFIELD(h264PictureInfo, pd, nNumSlices);
    COPYFIELD(h264PictureInfo, pd, pSliceDataOffsets);

    for (NvU32 i = 0; i < 16; i++)
    {
        NVH264DPBEntry *dpb_in = &h264Data->dpb[i];
        NvMediaReferenceFrameH264 *dpb_out = &h264PictureInfo->referenceFrames[i];
        nvxio::RefCountedFrameBuffer* picbuf = (nvxio::RefCountedFrameBuffer*)dpb_in->pPicBuf;

        COPYFIELD(dpb_out, dpb_in, FrameIdx);
        COPYFIELD(dpb_out, dpb_in, is_long_term);
        dpb_out->field_order_cnt[0] = dpb_in->FieldOrderCnt[0];
        dpb_out->field_order_cnt[1] = dpb_in->FieldOrderCnt[1];
        dpb_out->top_is_reference = !!(dpb_in->used_for_reference & 1);
        dpb_out->bottom_is_reference = !!(dpb_in->used_for_reference & 2);
        dpb_out->surface = picbuf ? picbuf->videoSurface : NULL;
    }
}

// Client callbacks

static void cbRelease(void*, NVDPicBuff *p)
{
    nvxio::RefCountedFrameBuffer * buffer = (nvxio::RefCountedFrameBuffer*)p;

    if (buffer->nRefs > 0)
        buffer->nRefs--;
}

static NvBool cbDecodePicture(void *ptr, NVDPictureData *pd)
{
    nvxio::SampleAppContext *ctx = (nvxio::SampleAppContext*)ptr;
    NvMediaStatus status;
    nvxio::RefCountedFrameBuffer *targetBuffer = NULL;
    NvMediaPictureInfoH264 picInfoH264;

    if (pd->pCurrPic)
    {
        NvMediaBitstreamBuffer bitStreamBuffer[1];
        SetParamsH264(pd, &picInfoH264);

        targetBuffer = (nvxio::RefCountedFrameBuffer *)pd->pCurrPic;

        bitStreamBuffer[0].bitstream = (NvU8 *)pd->pBitstreamData;
        bitStreamBuffer[0].bitstreamBytes = pd->nBitstreamDataLen;

        printf("DecodePicture %d Ptr:%p Surface:%p (stream ptr:%p size: %d)...\n",
                ctx->nPicNum, targetBuffer, targetBuffer->videoSurface, pd->pBitstreamData, pd->nBitstreamDataLen);
        ctx->nPicNum++;

        if (targetBuffer->videoSurface)
        {
            status = NvMediaVideoDecoderRender(ctx->decoder, targetBuffer->videoSurface,
                        (NvMediaPictureInfo *)&picInfoH264, 1, &bitStreamBuffer[0]);
            if (status != NVMEDIA_STATUS_OK)
            {
                printf("Decode Picture: Decode failed: %d\n", status);
                return NV_FALSE;
            }
        }
        else
        {
            printf("Decode Picture: Invalid target surface\n");
        }
    }
    else
    {
        printf("Decode Picture: No valid frame\n");
        return NV_FALSE;
    }

    return NV_TRUE;
}

static NvBool cbDisplayPicture(void *ptr, NVDPicBuff *p, NvS64)
{
    nvxio::SampleAppContext *ctx = (nvxio::SampleAppContext*)ptr;
    nvxio::RefCountedFrameBuffer* buffer = (nvxio::RefCountedFrameBuffer*)p;

    if (p)
    {
        ctx->frameSource->DisplayFrame(buffer);
    }
    else
    {
        printf("Display: Invalid buffer\n");
        return NV_FALSE;
    }

    return NV_TRUE;
}

static void cbAddRef(void*, NVDPicBuff *p)
{
    nvxio::RefCountedFrameBuffer* buffer = (nvxio::RefCountedFrameBuffer*)p;
    buffer->nRefs++;
}

static void cbUnhandledNALU(void*, const NvU8*, NvS32)
{
}

static NvS32 cbBeginSequence(void *ptr, const NVDSequenceInfo *pnvsi)
{
    nvxio::SampleAppContext *ctx = (nvxio::SampleAppContext*)ptr;

    const char* chroma[] =
    {
        "Monochrome",
        "4:2:0",
        "4:2:2",
        "4:4:4"
    };

    NvU32 decodeBuffers = pnvsi->nDecodeBuffers;
    NvMediaVideoDecoderAttributes attributes;

    if (pnvsi->eCodec != NVCS_H264)
    {
        printf("BeginSequence: Invalid codec type: %d\n", pnvsi->eCodec);
        return 0;
    }

    printf("BeginSequence: %dx%d (disp: %dx%d) codec: H264 decode buffers: %d aspect: %d:%d fps: %f chroma: %s\n",
           pnvsi->nCodedWidth, pnvsi->nCodedHeight, pnvsi->nDisplayWidth, pnvsi->nDisplayHeight,
           pnvsi->nDecodeBuffers, pnvsi->lDARWidth, pnvsi->lDARHeight,
           pnvsi->fFrameRate, pnvsi->nChromaFormat > 3 ? "Invalid" : chroma[pnvsi->nChromaFormat]);

    if (ctx->frameSource->configuration.frameWidth == (vx_uint32)-1)
        ctx->frameSource->configuration.frameWidth = pnvsi->nDisplayWidth;
    if (ctx->frameSource->configuration.frameHeight == (vx_uint32)-1)
        ctx->frameSource->configuration.frameHeight= pnvsi->nDisplayHeight;
    ctx->frameSource->configuration.fps = static_cast<uint>(pnvsi->fFrameRate);

    if (!ctx->aspectRatio && pnvsi->lDARWidth && pnvsi->lDARHeight)
    {
        double aspect = (float)pnvsi->lDARWidth / (float)pnvsi->lDARHeight;
        if (aspect > 0.3 && aspect < 3.0)
            ctx->aspectRatio = aspect;
    }

    // Check resolution change
    if (pnvsi->nCodedWidth != ctx->decodeWidth || pnvsi->nCodedHeight != ctx->decodeHeight)
    {
        NvMediaVideoCodec codec;
        NvMediaSurfaceType surfType;
        NvU32 maxReferences;

        printf("BeginSequence: Resolution changed: Old:%dx%d New:%dx%d\n",
               ctx->decodeWidth, ctx->decodeHeight, pnvsi->nCodedWidth, pnvsi->nCodedHeight);

        ctx->decodeWidth = pnvsi->nCodedWidth;
        ctx->decodeHeight = pnvsi->nCodedHeight;

        ctx->displayWidth = pnvsi->nDisplayWidth;
        ctx->displayHeight = pnvsi->nDisplayHeight;

        if (ctx->decoder)
        {
            NvMediaVideoDecoderDestroy(ctx->decoder);
        }

        printf("Create decoder: ");
        codec = NVMEDIA_VIDEO_CODEC_H264;
        printf("NVMEDIA_VIDEO_CODEC_H264");

        maxReferences = (decodeBuffers > 0) ? decodeBuffers - 1 : 0;
        maxReferences = (maxReferences > 16) ? 16 : maxReferences;

        printf(" Size: %dx%d maxReferences: %d\n", ctx->decodeWidth, ctx->decodeHeight,
               maxReferences);
        ctx->decoder = NvMediaVideoDecoderCreate(
            codec,                   // codec
            ctx->decodeWidth,        // width
            ctx->decodeHeight,       // height
            maxReferences,           // maxReferences
            pnvsi->MaxBitstreamSize, //maxBitstreamSize
            5);                      // inputBuffering
        if (!ctx->decoder)
        {
            printf("Unable to create decoder\n");
            return 0;
        }

        //set progressive sequence
        attributes.progressiveSequence = pnvsi->bProgSeq;
        NvMediaVideoDecoderSetAttributes(
            ctx->decoder,
            NVMEDIA_VIDEO_DECODER_ATTRIBUTE_PROGRESSIVE_SEQUENCE,
            &attributes);

        for(int i = 0; i < MAX_FRAMES; i++)
        {
            if (ctx->RefFrame[i].videoSurface)
            {
                NvMediaVideoSurfaceDestroy(ctx->RefFrame[i].videoSurface);
            }
        }

        memset(&ctx->RefFrame[0], 0, sizeof(nvxio::RefCountedFrameBuffer) * MAX_FRAMES);

        switch (pnvsi->nChromaFormat)
        {
            case 0: // Monochrome
            case 1: // 4:2:0
                printf("Chroma format: NvMediaSurfaceType_YV12\n");
                surfType = NvMediaSurfaceType_YV12;
                break;
            case 2: // 4:2:2
                printf("Chroma format: NvMediaSurfaceType_YV16\n");
                surfType = NvMediaSurfaceType_YV16;
                break;
            case 3: // 4:4:4
                printf("Chroma format: NvMediaSurfaceType_YV24\n");
                surfType = NvMediaSurfaceType_YV24;
                break;
            default:
                printf("Invalid chroma format: %d\n", pnvsi->nChromaFormat);
                return 0;
        }

        ctx->producer = NvMediaEglStreamProducerCreate(ctx->device, ctx->eglDisplay, ctx->eglStream,
                                                       ctx->surfaceType, ctx->displayWidth, ctx->displayHeight);
        if(!ctx->producer)
        {
            printf("Unable to create producer\n");
            return 0;
        }

        for (int i = 0; i < MAX_RENDER_SURFACE; i++)
        {
            ctx->renderSurfaces[i] = NvMediaVideoSurfaceCreate(
                ctx->device,
                ctx->surfaceType,
                ctx->displayWidth,
                ctx->displayHeight);
            if(!ctx->renderSurfaces[i])
            {
                printf("Unable to create render surface\n");
                return 0;
            }
            ctx->freeRenderSurfaces[i] = ctx->renderSurfaces[i];
        }

        ctx->nBuffers = decodeBuffers + MAX_DISPLAY_BUFFERS;

        for(int i = 0; i < ctx->nBuffers; i++)
        {
            ctx->RefFrame[i].videoSurface =
            NvMediaVideoSurfaceCreate(
                ctx->device,
                surfType,
                (pnvsi->nCodedWidth + 15) & ~15,
                                      (pnvsi->nCodedHeight + 15) & ~15);
            if (!ctx->RefFrame[i].videoSurface)
            {
                printf("Unable to create video surface\n");
                return 0;
            }
            printf("Create video surface[%d]: %dx%d\n Ptr:%p Surface:%p Device:%p\n", i,
                   (pnvsi->nCodedWidth + 15) & ~15, (pnvsi->nCodedHeight + 15) & ~15,
                   &ctx->RefFrame[i], ctx->RefFrame[i].videoSurface, ctx->device);
        }

        ctx->frameSource->VideoMixerDestroy();
        ctx->frameSource->VideoMixerInit(ctx->displayWidth, ctx->displayHeight,
                            pnvsi->nCodedWidth, pnvsi->nCodedHeight);
    }
    else
    {
        printf("cbBeginSequence: No resolution change\n");
    }

    // Syncronization
    {
        std::lock_guard<std::mutex> lock(ctx->mutex);
        ctx->isStarted = true;
    }
    ctx->condVariable.notify_one();

    return decodeBuffers;
}

static NvBool cbAllocPictureBuffer(void *ptr, NVDPicBuff **p)
{
    printf("\tcbAllocPictureBuffer\n");
    nvxio::SampleAppContext *ctx = (nvxio::SampleAppContext*)ptr;
    *p = (NVDPicBuff *) NULL;

    for (int i = 0; i < ctx->nBuffers; i++)
    {
        if (!ctx->RefFrame[i].nRefs)
        {
            *p = (NVDPicBuff *) &ctx->RefFrame[i];
            ctx->RefFrame[i].nRefs++;
            printf("Alloc picture index: %d Ptr:%p Surface:%p\n", i, *p, ctx->RefFrame[i].videoSurface);
            return NV_TRUE;
        }
    }

    printf("Alloc picture failed\n");
    return NV_FALSE;
}

static NVDClientCb TestClientCb =
{
    &cbBeginSequence,
    &cbDecodePicture,
    &cbDisplayPicture,
    &cbUnhandledNALU,
    &cbAllocPictureBuffer,
    &cbRelease,
    &cbAddRef
};

namespace nvxio
{

NvMediaFrameSourceImpl::NvMediaFrameSourceImpl(vx_context vxContext, const std::string path) :
    context()
{
    memset(&context, 0, sizeof(context));

    context.vxContext = vxContext;
    filePath = path;

    context.eglStream = EGL_NO_STREAM_KHR;
    context.cudaConsumer = 0;
    context.eglDisplay = EGL_NO_DISPLAY;
    context.alive = false;

    context.frameSource = this;
    scaledImage = NULL;
}

bool NvMediaFrameSourceImpl::InitializeEGLDisplay()
{
    // Obtain the EGL display
    context.eglDisplay = nvxio::EGLDisplayAccessor::getInstance();
    if (context.eglDisplay == EGL_NO_DISPLAY)
    {
        printf("EGL failed to obtain display.\n");
        return false;
    }

    return true;
}

EGLStreamKHR NvMediaFrameSourceImpl::InitializeEGLStream()
{
    static const EGLint streamAttrFIFOMode[] = { EGL_STREAM_FIFO_LENGTH_KHR, 4, EGL_NONE };
    EGLint fifo_length = 4, latency = 0, timeout = 0;
    EGLStreamKHR stream = EGL_NO_STREAM_KHR;

    if(!setupExtensions())
    {
        printf("Couldn't setup EGL extensions.\n");
        return EGL_NO_STREAM_KHR;
    }

    stream = eglCreateStreamKHR(context.eglDisplay, streamAttrFIFOMode);
    if (stream == EGL_NO_STREAM_KHR)
    {
        printf("Couldn't create stream.\n");
        return EGL_NO_STREAM_KHR;
    }

    // Set stream attribute
    if(!eglStreamAttribKHR(context.eglDisplay, stream, EGL_CONSUMER_LATENCY_USEC_KHR, 16000))
    {
        printf("Consumer: streamAttribKHR EGL_CONSUMER_LATENCY_USEC_KHR failed\n");
    }
    if(!eglStreamAttribKHR(context.eglDisplay, stream, EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR, 16000))
    {
        printf("Consumer: streamAttribKHR EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR failed\n");
    }

    // Get stream attributes
    if(!eglQueryStreamKHR(context.eglDisplay, stream, EGL_STREAM_FIFO_LENGTH_KHR, &fifo_length))
    {
        printf("Consumer: eglQueryStreamKHR EGL_STREAM_FIFO_LENGTH_KHR failed\n");
    }
    if(!eglQueryStreamKHR(context.eglDisplay, stream, EGL_CONSUMER_LATENCY_USEC_KHR, &latency))
    {
        printf("Consumer: eglQueryStreamKHR EGL_CONSUMER_LATENCY_USEC_KHR failed\n");
    }
    if(!eglQueryStreamKHR(context.eglDisplay, stream, EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR, &timeout))
    {
        printf("Consumer: eglQueryStreamKHR EGL_CONSUMER_ACQUIRE_TIMEOUT_USEC_KHR failed\n");
    }

    printf("EGL Stream consumer - Mode: FIFO Length: %d\n",  fifo_length);

    printf("EGL stream handle %p\n", stream);
    printf("EGL Stream consumer - Latency: %d usec\n", latency);
    printf("EGL Stream consumer - Timeout: %d usec\n", timeout);

    return stream;
}

void NvMediaFrameSourceImpl::FinalizeEglStream()
{
    if (context.eglStream != EGL_NO_STREAM_KHR)
    {
        eglDestroyStreamKHR(context.eglDisplay, context.eglStream);
        context.eglStream = EGL_NO_STREAM_KHR;
    }
}


bool NvMediaFrameSourceImpl::InitializeEglCudaConsumer()
{
    // Dummy CUDA call to initialize CUDA runtime context
    cudaFree(NULL);

    printf("Connect CUDA consumer\n");
    CUresult cuStatus = cuEGLStreamConsumerConnect(&context.cudaConsumer, context.eglStream);
    if (CUDA_SUCCESS != cuStatus)
    {
        printf("Connect CUDA consumer ERROR %d\n", cuStatus);
        return false;
    }

    return true;
}

void NvMediaFrameSourceImpl::FinalizeEglCudaConsumer()
{
    if (context.cudaConsumer)
    {
        cuEGLStreamConsumerDisconnect(&context.cudaConsumer);
        context.cudaConsumer = 0;
    }
}


bool NvMediaFrameSourceImpl::InitializeDecoder()
{
    float defaultFrameRate = 30.0;
    context.surfaceType =  NvMediaSurfaceType_R8G8B8A8;

    memset(&context.nvsi, 0, sizeof(context.nvsi));

    // create video context
    memset(&context.nvdp, 0, sizeof(NVDParserParams));
    context.nvdp.pClient = &TestClientCb;
    context.nvdp.pClientCtx = &context;
    context.nvdp.lErrorThreshold = 50;
    context.nvdp.lReferenceClockRate = 0;
    context.nvdp.eCodec = NVCS_H264;

    context.ctx = video_parser_create(&context.nvdp);
    if (!context.ctx)
    {
        printf("video_parser_create failed\n");
        return false;
    }

    if (!video_parser_set_attribute(context.ctx,
                               NVDVideoParserAttribute_SetDefaultFramerate,
                               sizeof(float), &defaultFrameRate))
    {
        printf("Failed to setup NVDVideoParserAttribute_SetDefaultFramerate\n");
        return false;
    }

    context.device = NvMediaDeviceCreate();
    if (!context.device)
    {
        printf("Unable to create device\n");
        return false;
    }

    return true;
}

void NvMediaFrameSourceImpl::FinalizeDecoder()
{
    if (context.ctx)
    {
        video_parser_destroy(context.ctx);
        context.ctx = NULL;
    }
    DisplayFlush();

    for(NvU32 i = 0; i < MAX_FRAMES; i++)
    {
        if (context.RefFrame[i].videoSurface)
        {
            NvMediaVideoSurfaceDestroy(context.RefFrame[i].videoSurface);
            context.RefFrame[i].videoSurface = NULL;
        }
    }

    if (context.decoder)
    {
        NvMediaVideoDecoderDestroy(context.decoder);
        context.decoder = NULL;
    }

    DisplayFlush();

    VideoMixerDestroy();

    for (NvU32 i = 0; i < MAX_RENDER_SURFACE; i++)
    {
        if(context.renderSurfaces[i])
        {
            NvMediaVideoSurfaceDestroy(context.renderSurfaces[i]);
            context.renderSurfaces[i] = NULL;
        }
    }

    if(context.producer)
    {
        NvMediaEglStreamProducerDestroy(context.producer);
        context.producer = NULL;
    }

    if (context.device)
    {
        NvMediaDeviceDestroy(context.device);
        context.device = NULL;
    }
}

void NvMediaFrameSourceImpl::FetchVideoFile()
{
    fetchThread = std::thread( [&] ()
    {
        NvU8 * bits = NULL;
        NvU32 readSize = READ_SIZE;
        FILE * fp = NULL;

        fp = fopen(filePath.c_str(), "rb");
        if (!fp)
        {
            printf("Failed to open stream %s\n", filePath.c_str());
            return;
        }

        bits = (NvU8*)malloc(readSize);
        if (!bits)
        {
            fclose(fp);

            printf("Cannot allocate memory for file reading\n");
            return;
        }

        context.alive = true;

        while (!feof(fp) && context.alive)
        {
            size_t len;
            bitstream_packet_s packet;

            memset(&packet, 0, sizeof(bitstream_packet_s));

            len = fread(bits, 1, readSize, fp);
            if (len == 0)
            {
                printf("Failed to read from the source %s\n", filePath.c_str());
                context.alive = false;
                fclose(fp);
                free(bits);

                return;
            }

            packet.nDataLength = (NvS32) len;
            packet.pByteStream = bits;
            packet.bEOS = feof(fp);
            packet.bPTSValid = 0;
            packet.llPTS = 0;
            printf("flushing\n");

            if (!video_parser_parse(context.ctx, &packet))
            {
                printf("Cannot parse video\n");
                context.alive = false;
                fclose(fp);
                free(bits);

                return;
            }
        }

        video_parser_flush(context.ctx);
        DisplayFlush();

        free(bits);
        fclose(fp);

        context.alive = false;
    });

    std::unique_lock<std::mutex> lock(context.mutex);
    context.condVariable.wait(lock, [&] () { return context.isStarted; } );
}


bool NvMediaFrameSourceImpl::VideoMixerInit(int width, int height, int videoWidth, int videoHeight)
{
    unsigned int features =  0;
    float aspectRatio = (float)width / (float)height;

    if (context.aspectRatio != 0.0)
    {
        aspectRatio = context.aspectRatio;
    }

    printf("VideoMixerInit: %dx%d Aspect: %f\n", width, height, aspectRatio);

    /* default Deinterlace: Off/Weave */
    printf("VideoMixerInit: Surface Renderer Mixer create\n");
    features |= (NVMEDIA_VIDEO_MIXER_FEATURE_DVD_MIXING_MODE | NVMEDIA_VIDEO_MIXER_FEATURE_SURFACE_RENDERING);

    context.mixer = NvMediaVideoMixerCreate(
        context.device,       // device,
        width,                // mixerWidth
        height,               // mixerHeight
        aspectRatio,          // sourceAspectRatio
        videoWidth,           // primaryVideoWidth
        videoHeight,          // primaryVideoHeight
        0,                    // secondaryVideoWidth
        0,                    // secondaryVideoHeight
        0,                    // graphics0Width
        0,                    // graphics0Height
        0,                    // graphics1Width
        0,                    // graphics1Height
        features,
        NULL);

    if (!context.mixer)
    {
        printf("Unable to create mixer\n");
        return false;
    }

    printf("VideoMixerInit: Mixer:%p\n", context.mixer);

    return true;
}

void NvMediaFrameSourceImpl::VideoMixerDestroy()
{
    if (context.mixer)
    {
        NvMediaVideoMixerDestroy(context.mixer);
        context.mixer = NULL;
    }
}


void NvMediaFrameSourceImpl::ReleaseFrame(NvMediaVideoSurface *videoSurface)
{
    for (int i = 0; i < MAX_FRAMES; i++)
    {
        if (videoSurface == context.RefFrame[i].videoSurface)
        {
            cbRelease((void *)&context, (NVDPicBuff *)&context.RefFrame[i]);
            break;
        }
    }
}

void NvMediaFrameSourceImpl::DisplayFlush()
{
    NvMediaVideoSurface *renderSurface = NULL;

    if (context.producer)
    {
        printf("before NvMediaEglStreamProducerGetSurface loop\n");
        while (NvMediaEglStreamProducerGetSurface(context.producer, &renderSurface, 0) == NVMEDIA_STATUS_OK)
        {
            printf("NvMediaEglStreamProducerGetSurface iteration\n");
            if ((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
                (context.surfaceType == NvMediaSurfaceType_R8G8B8A8))
            {
                for (int i = 0; i < MAX_RENDER_SURFACE; i++)
                {
                    if(!context.freeRenderSurfaces[i])
                    {
                        context.freeRenderSurfaces[i] = renderSurface;
                        break;
                    }
                }
            }
            else
            {
                ReleaseFrame(renderSurface);
            }
        }
    }
}


NvMediaVideoSurface * NvMediaFrameSourceImpl::GetRenderSurface()
{
    NvMediaVideoSurface *renderSurface = NULL;

    if ((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
        (context.surfaceType == NvMediaSurfaceType_R8G8B8A8))
    {
        for (int i = 0; i < MAX_RENDER_SURFACE; i++)
        {
            if (context.freeRenderSurfaces[i])
            {
                renderSurface = context.freeRenderSurfaces[i];
                context.freeRenderSurfaces[i] = NULL;
                return renderSurface;
            }
        }
    }

    while (context.alive)
    {
        NvMediaStatus status = NvMediaEglStreamProducerGetSurface(context.producer, &renderSurface, 50);
        if (status == NVMEDIA_STATUS_ERROR &&
           ((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
           (context.surfaceType == NvMediaSurfaceType_R8G8B8A8)))
        {
            printf("GetRenderSurface: NvMediaGetSurface waiting\n");
        }

        // EGL stream producer was able to get a free surface
        if (renderSurface)
        {
            return renderSurface;
        }
    }

    return renderSurface;
}

// push to EGL stream
void NvMediaFrameSourceImpl::DisplayFrame(RefCountedFrameBuffer *frame)
{
    NvMediaPrimaryVideo primaryVideo;
    NvMediaStatus status;
    NvMediaRect primarySourceRect = { 0, 0, (unsigned short)context.displayWidth,
                                            (unsigned short)context.displayHeight };

    if (!frame || !frame->videoSurface)
    {
        printf("DisplayFrame: Invalid surface\n");
        return;
    }

    NvMediaVideoSurface *renderSurface = GetRenderSurface();
    if (!renderSurface)
    {
        if ((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
            (context.surfaceType == NvMediaSurfaceType_R8G8B8A8))
        {
            printf("DisplayFrame: GetRenderSurface empty\n");
            return;
        }
    }

    /* Deinterlace Off/Weave */
    primaryVideo.pictureStructure = NVMEDIA_PICTURE_STRUCTURE_FRAME;
    primaryVideo.next = NULL;
    primaryVideo.current = frame->videoSurface;
    primaryVideo.previous = NULL;
    primaryVideo.previous2 = NULL;
    primaryVideo.srcRect = &primarySourceRect;
    primaryVideo.dstRect = NULL;

    frame->nRefs++;

    if ((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
        (context.surfaceType == NvMediaSurfaceType_R8G8B8A8))
    {
        // Render to surface
        printf("DisplayFrame: Render to surface\n");
        status = NvMediaVideoMixerRenderSurface(
            context.mixer, // mixer
            renderSurface, // renderSurface
            NULL,          // background
            &primaryVideo, // primaryVideo
            NULL,          // secondaryVideo
            NULL,          // graphics0
            NULL);         // graphics1
        if (status != NVMEDIA_STATUS_OK)
        {
            printf("DisplayFrame: NvMediaVideoMixerRender failed\n");
        }
    }

    status = NvMediaEglStreamProducerPostSurface(context.producer,
                                        (context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
                                        (context.surfaceType == NvMediaSurfaceType_R8G8B8A8) ?
                                        renderSurface : frame->videoSurface,
                                        NULL);
    if (status != NVMEDIA_STATUS_OK)
    {
        printf("DisplayFrame: NvMediaEglStreamProducerPostSurface failed\n");
    }

    ReleaseFrame((context.surfaceType == NvMediaSurfaceType_R8G8B8A8_BottomOrigin) ||
                         (context.surfaceType == NvMediaSurfaceType_R8G8B8A8) ?
                         primaryVideo.current : renderSurface);
}


bool NvMediaFrameSourceImpl::open()
{
    close();

    printf("Initializing EGL display\n");
    if (!InitializeEGLDisplay())
    {
        printf("Cannot initialize EGL display\n");
        return false;
    }

    printf("Initializing EGL stream\n");
    context.eglStream = InitializeEGLStream();
    if (context.eglStream == EGL_NO_STREAM_KHR)
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

    printf("Initializing MvMedia Decoder\n");
    if (!InitializeDecoder())
    {
        printf("Cannot initialize NvMedia decoder\n");
        return false;
    }

    // fetching
    FetchVideoFile();

    // reset the timer
    lastFrameTimestamp.tic();

    return true;
}

FrameSource::FrameStatus NvMediaFrameSourceImpl::fetch(vx_image image, vx_uint32 timeout /*milliseconds*/)
{
    if (!context.alive)
    {
        close();
        return FrameSource::FrameStatus::CLOSED;
    }

    CUresult cuStatus;
    CUgraphicsResource cudaResource;

    EGLint streamState = 0;
    if (!eglQueryStreamKHR(context.eglDisplay, context.eglStream, EGL_STREAM_STATE_KHR, &streamState))
    {
        printf("Cuda consumer, eglQueryStreamKHR EGL_STREAM_STATE_KHR failed\n");
        return FrameSource::FrameStatus::CLOSED;
    }

    if (streamState == EGL_STREAM_STATE_DISCONNECTED_KHR)
    {
        printf("CUDA Consumer: - EGL_STREAM_STATE_DISCONNECTED_KHR received\n");
        return FrameSource::FrameStatus::CLOSED;
    }

    if (streamState != EGL_STREAM_STATE_NEW_FRAME_AVAILABLE_KHR)
    {
        if ((lastFrameTimestamp.toc()/1000.0) > Application::get().getSourceDefaultTimeout())
        {
            close();
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

    cuStatus = cuEGLStreamConsumerAcquireFrame(&context.cudaConsumer, &cudaResource, NULL, timeout);
    if (cuStatus != CUDA_SUCCESS)
    {
        printf("Cuda Acquire failed cuStatus=%d\n", cuStatus);

        return FrameSource::FrameStatus::TIMEOUT;
    }

    CUeglFrame eglFrame;
    cuStatus = cuGraphicsResourceGetMappedEglFrame(&eglFrame, cudaResource, 0, 0);
    if (cuStatus != CUDA_SUCCESS)
    {
        const char* error;
        cuGetErrorString(cuStatus, &error);
        printf("Cuda get resource failed with error: \"%s\"\n", error);
        return FrameSource::FrameStatus::CLOSED;
    }

    vx_imagepatch_addressing_t srcImageAddr;
    srcImageAddr.dim_x = eglFrame.width;
    srcImageAddr.dim_y = eglFrame.height;
    srcImageAddr.stride_x = 4;
    srcImageAddr.stride_y = eglFrame.pitch;
    srcImageAddr.scale_x = 1;
    srcImageAddr.scale_y = 1;

    void *ptrs[] = { eglFrame.frame.pPitch[0] };
    vx_image srcImage = vxCreateImageFromHandle(context.vxContext, VX_DF_IMAGE_RGBX, &srcImageAddr,
                                                ptrs, NVX_IMPORT_TYPE_CUDA);
    NVXIO_CHECK_REFERENCE( srcImage );

    // fetch image width and height
    vx_uint32 actual_width, actual_height;
    vx_df_image_e actual_format;
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_WIDTH, (void *)&actual_width, sizeof(actual_width)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_HEIGHT, (void *)&actual_height, sizeof(actual_height)) );
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT, (void *)&actual_format, sizeof(actual_format)) );
    bool needScale = eglFrame.width != configuration.frameWidth ||
            eglFrame.height != configuration.frameHeight;

    // config and actual image sized must be the same!
    if ((actual_height != configuration.frameHeight) ||
            (actual_width != configuration.frameWidth) ||
            (actual_format != configuration.format))
    {
        cuStatus = cuEGLStreamConsumerReleaseFrame(&context.cudaConsumer, cudaResource, NULL);
        close();
        NVXIO_THROW_EXCEPTION("Actual image [ " << actual_width << " x " << actual_height <<
                              " ] does not equal configuration one [ " << configuration.frameWidth
                              << " x " << configuration.frameHeight << " ]");
    }

    if (needScale && !scaledImage &&
            (configuration.format != VX_DF_IMAGE_RGBX))
    {
        scaledImage = vxCreateImage(context.vxContext, configuration.frameWidth,
                                    configuration.frameHeight, VX_DF_IMAGE_RGBX);
        NVXIO_CHECK_REFERENCE( scaledImage );
    }

    if (needScale)
    {
        NVXIO_SAFE_CALL( vxuScaleImage(context.vxContext, srcImage, scaledImage ? scaledImage : image, VX_INTERPOLATION_TYPE_BILINEAR) );
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
            NVXIO_SAFE_CALL( nvxuCopyImage(context.vxContext, scaledImage, image) );
        }
        break;
    case VX_DF_IMAGE_RGB:
        NVXIO_SAFE_CALL( vxuColorConvert(context.vxContext, scaledImage, image) );
        break;
    case VX_DF_IMAGE_U8:
        NVXIO_SAFE_CALL( vxuColorConvert(context.vxContext, scaledImage, image) );
        break;
    }

    if (!needScale)
    {
        scaledImage = NULL;
    }

    vxReleaseImage(&srcImage);

    cuStatus = cuEGLStreamConsumerReleaseFrame(&context.cudaConsumer, cudaResource, NULL);
    if (cuStatus != CUDA_SUCCESS)
    {
        printf("Failed to release EGL frame\n");
        close();
        return FrameSource::FrameStatus::CLOSED;
    }

    return FrameSource::FrameStatus::OK;
}

FrameSource::Parameters NvMediaFrameSourceImpl::getConfiguration()
{
    return configuration;
}

bool NvMediaFrameSourceImpl::setConfiguration(const FrameSource::Parameters& params)
{
    bool result = true;

    if (context.alive)
    {
        if ((params.frameWidth != (vx_uint32)-1) && (params.frameWidth != configuration.frameWidth))
            result = false;
        if ((params.frameHeight != (vx_uint32)-1) && (params.frameHeight != configuration.frameHeight))
            result = false;
    }
    else
    {
        configuration.frameHeight = params.frameHeight;
        configuration.frameWidth = params.frameWidth;
    }

    if ((params.fps != (vx_uint32)-1) && (params.fps != configuration.fps))
        result = false;

    configuration.format = params.format;

    return result;
}

void NvMediaFrameSourceImpl::close()
{
    context.alive = false;
    context.isStarted = false;

    if (fetchThread.joinable())
        fetchThread.join();

    printf("Finalizing EGL CUDA consumer\n");
    FinalizeEglCudaConsumer();

    printf("Finalizing NvMedia decoder\n");
    FinalizeDecoder();

    printf("Finalizing EGL Stream\n");
    FinalizeEglStream();

    context.decodeWidth = -1;
    context.decodeHeight = -1;

    if (scaledImage)
    {
        vxReleaseImage(&scaledImage);
        scaledImage = NULL;
    }
}

NvMediaFrameSourceImpl::~NvMediaFrameSourceImpl()
{
    close();
}

}

#endif // USE_NVMEDIA
