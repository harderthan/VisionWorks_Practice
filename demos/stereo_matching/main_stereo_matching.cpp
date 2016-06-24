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

#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <memory>

#include <NVX/nvx.h>
#include <NVX/nvx_timer.hpp>

#include "stereo_matching.hpp"
#include <NVXIO/Application.hpp>
#include <NVXIO/FrameSource.hpp>
#include <NVXIO/Render.hpp>
#include <NVXIO/Utility.hpp>
#include <NVXIO/ConfigParser.hpp>

//
// Color disparity graph
//

class ColorDisparityGraph
{
public:
    ColorDisparityGraph(vx_context context, vx_image disparity, vx_image output, vx_int32 ndisp);
    ~ColorDisparityGraph();

    void process();
    void printPerfs();

private:
    static void fillLUT(vx_lut r_lut, vx_lut g_lut, vx_lut b_lut, vx_int32 ndisp);

    vx_graph graph_;
    vx_node lut_node_[3];
    vx_node combine_node_;
};

ColorDisparityGraph::ColorDisparityGraph(vx_context context, vx_image disparity, vx_image output, vx_int32 ndisp) :
    graph_(nullptr)
{
    NVXIO_ASSERT(ndisp <= 256);

    vx_lut r_lut = vxCreateLUT(context, VX_TYPE_UINT8, 256);
    vx_lut g_lut = vxCreateLUT(context, VX_TYPE_UINT8, 256);
    vx_lut b_lut = vxCreateLUT(context, VX_TYPE_UINT8, 256);
    fillLUT(r_lut, g_lut, b_lut, ndisp);

    graph_ = vxCreateGraph(context);

    vx_image r_img = vxCreateVirtualImage(graph_, 0, 0, VX_DF_IMAGE_U8);
    vx_image g_img = vxCreateVirtualImage(graph_, 0, 0, VX_DF_IMAGE_U8);
    vx_image b_img = vxCreateVirtualImage(graph_, 0, 0, VX_DF_IMAGE_U8);

    lut_node_[0] = vxTableLookupNode(graph_, disparity, r_lut, r_img);
    lut_node_[1] = vxTableLookupNode(graph_, disparity, g_lut, g_img);
    lut_node_[2] = vxTableLookupNode(graph_, disparity, b_lut, b_img);

    combine_node_ = vxChannelCombineNode(graph_, r_img, g_img, b_img, nullptr, output);

    NVXIO_SAFE_CALL( vxVerifyGraph(graph_) );

    vxReleaseImage(&r_img);
    vxReleaseImage(&g_img);
    vxReleaseImage(&b_img);

    vxReleaseLUT(&r_lut);
    vxReleaseLUT(&g_lut);
    vxReleaseLUT(&b_lut);
}

void ColorDisparityGraph::fillLUT(vx_lut r_lut, vx_lut g_lut, vx_lut b_lut, vx_int32 ndisp)
{
    vx_uint8* r_lut_ptr = nullptr;
    vx_uint8* g_lut_ptr = nullptr;
    vx_uint8* b_lut_ptr = nullptr;
    NVXIO_SAFE_CALL( vxAccessLUT(r_lut, (void **)&r_lut_ptr, VX_WRITE_ONLY) );
    NVXIO_SAFE_CALL( vxAccessLUT(g_lut, (void **)&g_lut_ptr, VX_WRITE_ONLY) );
    NVXIO_SAFE_CALL( vxAccessLUT(b_lut, (void **)&b_lut_ptr, VX_WRITE_ONLY) );

    for (vx_int32 d = 0; d < 256; ++d)
    {
        vx_int32 H = ((ndisp - d) * 240) / ndisp;
        vx_float32 S = 1.0f;
        vx_float32 V = 1.0f;

        vx_int32 hi = (H / 60) % 6;
        vx_float32 f = H / 60.0f - H / 60;
        vx_float32 p = V * (1.0f - S);
        vx_float32 q = V * (1.0f - f * S);
        vx_float32 t = V * (1.0f - (1 - f) * S);

        vx_float32 rval = 0.0f, gval = 0.0f, bval = 0.0f;

        if (hi == 0) //R = V, G = t, B = p
        {
            bval = p;
            gval = t;
            rval = V;
        }
        if (hi == 1) // R = q, G = V, B = p
        {
            bval = p;
            gval = V;
            rval = q;
        }
        if (hi == 2) // R = p, G = V, B = t
        {
            bval = t;
            gval = V;
            rval = p;
        }
        if (hi == 3) // R = p, G = q, B = V
        {
            bval = V;
            gval = q;
            rval = p;
        }
        if (hi == 4) // R = t, G = p, B = V
        {
            bval = V;
            gval = p;
            rval = t;
        }
        if (hi == 5) // R = V, G = p, B = q
        {
            bval = q;
            gval = p;
            rval = V;
        }

        r_lut_ptr[d] = std::max(0.f, std::min(rval, 1.f)) * 255.f;
        g_lut_ptr[d] = std::max(0.f, std::min(gval, 1.f)) * 255.f;
        b_lut_ptr[d] = std::max(0.f, std::min(bval, 1.f)) * 255.f;
    }

    vxCommitLUT(r_lut, r_lut_ptr);
    vxCommitLUT(g_lut, g_lut_ptr);
    vxCommitLUT(b_lut, b_lut_ptr);
}

ColorDisparityGraph::~ColorDisparityGraph()
{
    vxReleaseGraph(&graph_);
}

void ColorDisparityGraph::process()
{
    NVXIO_SAFE_CALL( vxProcessGraph(graph_) );
}

void ColorDisparityGraph::printPerfs()
{
    vx_perf_t perf;

    NVXIO_SAFE_CALL( vxQueryGraph(graph_, VX_GRAPH_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf)) );
    std::cout << "Color Disparity Graph Time : " << perf.tmp / 1000000.0 << " ms" << std::endl;

    NVXIO_SAFE_CALL( vxQueryNode(lut_node_[0], VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf)) );
    std::cout << "\t Red Channel Table Lookup : " << perf.tmp / 1000000.0 << " ms" << std::endl;

    NVXIO_SAFE_CALL( vxQueryNode(lut_node_[1], VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf)) );
    std::cout << "\t Green Channel Table Lookup : " << perf.tmp / 1000000.0 << " ms" << std::endl;

    NVXIO_SAFE_CALL( vxQueryNode(lut_node_[2], VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf)) );
    std::cout << "\t Blue Channel Table Lookup : " << perf.tmp / 1000000.0 << " ms" << std::endl;

    NVXIO_SAFE_CALL( vxQueryNode(combine_node_, VX_NODE_ATTRIBUTE_PERFORMANCE, &perf, sizeof(perf)) );
    std::cout << "\t Channel Combine Time : " << perf.tmp / 1000000.0 << " ms" << std::endl;
}

//
// Utility functions
//

static void displayState(nvxio::Render *renderer,
                         const nvxio::FrameSource::Parameters &sourceParams,
                         double proc_ms, double total_ms)
{
    std::ostringstream txt;

    txt << std::fixed << std::setprecision(1);

    nvxio::Render::TextBoxStyle style = {{255, 255, 255, 255}, {0, 0, 0, 255}, {10, 10}};

    txt << "Source size: " << sourceParams.frameWidth << 'x' << sourceParams.frameHeight / 2 << std::endl;
    txt << "Algorithm: " << proc_ms << " ms / " << 1000.0 / proc_ms << " FPS" << std::endl;
    txt << "Display: " << total_ms  << " ms / " << 1000.0 / total_ms << " FPS" << std::endl;

    txt << std::setprecision(6);
    txt.unsetf(std::ios_base::floatfield);
    txt << "LIMITED TO " << nvxio::Application::get().getFPSLimit() << " FPS FOR DISPLAY" << std::endl;
    txt << "S - switch Frame / Disparity / Color output" << std::endl;
    txt << "Space - pause/resume" << std::endl;
    txt << "Esc - close the demo" << std::endl;
    renderer->putText(txt.str(), style);
}

static bool read(const std::string &nf, StereoMatching::StereoMatchingParams &config, std::string &message)
{
    std::unique_ptr<nvxio::ConfigParser> parser(nvxio::createConfigParser());
    parser->addParameter("min_disparity",
                         nvxio::OptionHandler::integer(
                             &config.min_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("max_disparity",
                         nvxio::OptionHandler::integer(
                             &config.max_disparity,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P1",
                         nvxio::OptionHandler::integer(
                             &config.P1,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("P2",
                         nvxio::OptionHandler::integer(
                             &config.P2,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));
    parser->addParameter("sad",
                         nvxio::OptionHandler::integer(
                             &config.sad,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(31)));
    parser->addParameter("bt_clip_value",
                         nvxio::OptionHandler::integer(
                             &config.bt_clip_value,
                             nvxio::ranges::atLeast(15) & nvxio::ranges::atMost(95)));
    parser->addParameter("max_diff",
                         nvxio::OptionHandler::integer(
                             &config.max_diff));
    parser->addParameter("uniqueness_ratio",
                         nvxio::OptionHandler::integer(
                             &config.uniqueness_ratio,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(100)));
    parser->addParameter("scanlines_mask",
                         nvxio::OptionHandler::integer(
                             &config.scanlines_mask,
                             nvxio::ranges::atLeast(0) & nvxio::ranges::atMost(256)));

    message = parser->parse(nf);

    return message.empty();
}

//
// Process events
//

enum OUTPUT_IMAGE
{
    ORIG_FRAME,
    ORIG_DISPARITY,
    COLOR_OUTPUT
};

struct EventData
{
    EventData() : shouldStop(false), outputImg(COLOR_OUTPUT), pause(false) {}

    bool shouldStop;
    OUTPUT_IMAGE outputImg;
    bool pause;
};

static void eventCallback(void* eventData, vx_char key, vx_uint32, vx_uint32)
{
    EventData* data = static_cast<EventData*>(eventData);

    if (key == 27)
    {
        data->shouldStop = true;
    }
    else if (key == 's')
    {
        switch (data->outputImg)
        {
        case ORIG_FRAME:
            data->outputImg = ORIG_DISPARITY;
            break;

        case ORIG_DISPARITY:
            data->outputImg = COLOR_OUTPUT;
            break;

        case COLOR_OUTPUT:
            data->outputImg = ORIG_FRAME;
            break;
        }
    }
    else if (key == 32)
    {
        data->pause = !data->pause;
    }
}

//
// main - Application entry point
//

int main(int argc, char* argv[])
{
    try
    {
        nvxio::Application &app = nvxio::Application::get();

        //
        // Parse command line arguments
        //

        std::string sourceUri  = app.findSampleFilePath("left_right.mp4");

        std::string configFile = app.findSampleFilePath("stereo_matching_demo_config.ini");
        StereoMatching::StereoMatchingParams params;

        app.setDescription("This demo demonstrates Stereo Matching algorithm");
        app.addOption('s', "source", "Source URI", nvxio::OptionHandler::string(&sourceUri));
        app.addOption('c', "config", "Config file path", nvxio::OptionHandler::string(&configFile));

        app.init(argc, argv);

        //
        // Read and check input parameters
        //

        std::string error;
        if (!read(configFile, params, error))
        {
            std::cerr << error;
            return nvxio::Application::APP_EXIT_CODE_INVALID_VALUE;
        }

        //
        // Create OpenVX context
        //

        nvxio::ContextGuard context;

        //
        // Create a Frame Source
        //

        std::unique_ptr<nvxio::FrameSource> source(
            nvxio::createDefaultFrameSource(context, sourceUri));

        if (!source || !source->open())
        {
            std::cerr << "Can't open source URI " << sourceUri << std::endl;
            return nvxio::Application::APP_EXIT_CODE_NO_FRAMESOURCE;
        }

        nvxio::FrameSource::Parameters sourceParams = source->getConfiguration();

        if (sourceParams.frameHeight % 2 != 0)
        {
            std::cerr << "\"" << sourceUri.c_str()
                      << "\" has odd height (" << sourceParams.frameHeight
                      << "). This demo requires the source's height to be even." << std::endl;
            return nvxio::Application::APP_EXIT_CODE_INVALID_DIMENSIONS;
        }

        //
        // Create a Render
        //

        std::unique_ptr<nvxio::Render> renderer(nvxio::createDefaultRender(context,
            "Stereo Matching Demo", sourceParams.frameWidth, sourceParams.frameHeight / 2));

        if (!renderer)
        {
            std::cerr << "Can't create a renderer" << std::endl;
            return nvxio::Application::APP_EXIT_CODE_NO_RENDER;
        }

        EventData eventData;
        renderer->setOnKeyboardEventCallback(eventCallback, &eventData);

        //
        // Messages generated by the OpenVX framework will be processed by nvxio::stdoutLogCallback
        //

        vxRegisterLogCallback(context, &nvxio::stdoutLogCallback, vx_false_e);

        //
        // Create OpenVX Image to hold frames from video source
        //

        vx_image top_bottom = vxCreateImage
            (context, sourceParams.frameWidth, sourceParams.frameHeight, sourceParams.format);
        NVXIO_CHECK_REFERENCE(top_bottom);

        vx_rectangle_t left_rect { 0, 0, sourceParams.frameWidth, sourceParams.frameHeight / 2 };
        vx_image left  = vxCreateImageFromROI(top_bottom, &left_rect);
        NVXIO_CHECK_REFERENCE(left);
        vx_rectangle_t right_rect { 0, sourceParams.frameHeight / 2, sourceParams.frameWidth, sourceParams.frameHeight };
        vx_image right = vxCreateImageFromROI(top_bottom, &right_rect);
        NVXIO_CHECK_REFERENCE(right);

        vx_image disparity = vxCreateImage
            (context, sourceParams.frameWidth, sourceParams.frameHeight / 2, VX_DF_IMAGE_U8);
        NVXIO_CHECK_REFERENCE(disparity);

        vx_image color_output = vxCreateImage
            (context, sourceParams.frameWidth, sourceParams.frameHeight / 2, VX_DF_IMAGE_RGB);

        //
        // Create StereoMatching instance
        //

        std::unique_ptr<StereoMatching> stereo(StereoMatching::createStereoMatching(
            context, params, left, right, disparity));

        ColorDisparityGraph color_disp_graph(context, disparity, color_output, params.max_disparity);
        bool color_disp_update = true;

        //
        // Run processing loop
        //

        nvx::Timer totalTimer;
        totalTimer.tic();
        double proc_ms = 0;
        while (!eventData.shouldStop)
        {
            if (!eventData.pause)
            {
                nvxio::FrameSource::FrameStatus frameStatus;

                do
                {
                    frameStatus = source->fetch(top_bottom);
                }
                while(frameStatus == nvxio::FrameSource::TIMEOUT);

                if (frameStatus == nvxio::FrameSource::CLOSED)
                {
                    if (!source->open())
                    {
                        std::cerr << "Failed to reopen the source" << std::endl;
                        break;
                    }
                    continue;
                }

                //
                // Process
                //

                nvx::Timer procTimer;
                procTimer.tic();

                stereo->run();

                proc_ms = procTimer.toc();

                //
                // Print performance results
                //

                stereo->printPerfs();

                color_disp_update = true;
            }

            switch (eventData.outputImg)
            {
            case ORIG_FRAME:
                renderer->putImage(left);
                break;

            case ORIG_DISPARITY:
                renderer->putImage(disparity);
                break;

            case COLOR_OUTPUT:
                if (color_disp_update)
                {
                    color_disp_graph.process();
                    color_disp_graph.printPerfs();
                    color_disp_update = false;
                }
                renderer->putImage(color_output);
                break;
            }

            double total_ms = totalTimer.toc();

            std::cout << "Display Time : " << total_ms << " ms" << std::endl << std::endl;

            app.sleepToLimitFPS(total_ms);

            total_ms = totalTimer.toc();

            totalTimer.tic();

            displayState(renderer.get(), sourceParams, proc_ms, total_ms);

            if (!renderer->flush())
            {
                eventData.shouldStop = true;
            }
        }

        //
        // Release all objects
        //

        vxReleaseImage(&top_bottom);
        vxReleaseImage(&left);
        vxReleaseImage(&right);
        vxReleaseImage(&disparity);
        vxReleaseImage(&color_output);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return nvxio::Application::APP_EXIT_CODE_ERROR;
    }

    return nvxio::Application::APP_EXIT_CODE_SUCCESS;
}
