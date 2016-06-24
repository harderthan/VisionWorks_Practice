/*
# Copyright (c) 2014-2015, NVIDIA CORPORATION. All rights reserved.
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

#include "feature_tracker.hpp"
#include <NVXIO/Application.hpp>
#include <NVXIO/FrameSource.hpp>
#include <NVXIO/Render.hpp>
#include <NVXIO/Utility.hpp>
#include <NVXIO/ConfigParser.hpp>

//
// Utility functions
//

static void drawArrows(nvxio::Render *renderer, vx_array old_points, vx_array new_points)
{
    nvxio::Render::FeatureStyle featureStyle = { { 255, 0, 0, 255 }, 4.0f };
    nvxio::Render::LineStyle arrowStyle = {{0, 255, 0, 255}, 1};

    renderer->putArrows(old_points, new_points, arrowStyle);
    renderer->putFeatures(old_points, featureStyle);
}

//
// Process events
//

struct EventData
{
    EventData(): shouldStop(false), pause(false) {}

    bool shouldStop;
    bool pause;
};

static void eventCallback(void* eventData, vx_char key, vx_uint32, vx_uint32)
{
    EventData* data = static_cast<EventData*>(eventData);

    if (key == 27)
    {
        data->shouldStop = true;
    }
    else if (key == 32)
    {
        data->pause = !data->pause;
    }
}

static void displayState(nvxio::Render *renderer,
                         const nvxio::FrameSource::Parameters &sourceParams,
                         double proc_ms, double total_ms)
{
    std::ostringstream txt;

    txt << std::fixed << std::setprecision(1);

    nvxio::Render::TextBoxStyle style = {{255, 255, 255, 255}, {0, 0, 0, 255}, {10, 10}};

    txt << "Source size: " << sourceParams.frameWidth << 'x' << sourceParams.frameHeight << std::endl;
    txt << "Algorithm: " << proc_ms << " ms / " << 1000.0 / proc_ms << " FPS" << std::endl;
    txt << "Display: " << total_ms  << " ms / " << 1000.0 / total_ms << " FPS" << std::endl;

    txt << std::setprecision(6);
    txt.unsetf(std::ios_base::floatfield);
    txt << "LIMITED TO " << nvxio::Application::get().getFPSLimit() << " FPS FOR DISPLAY" << std::endl;

    txt << "Space - pause/resume" << std::endl;
    txt << "Esc - close the demo";
    renderer->putText(txt.str(), style);
}

static bool read(const std::string & nf, nvx::FeatureTracker::HarrisPyrLKParams &config, std::string &message)
{
    std::unique_ptr<nvxio::ConfigParser> ftparser(nvxio::createConfigParser());
    ftparser->addParameter("pyr_levels", nvxio::OptionHandler::unsignedInteger(&config.pyr_levels,
             nvxio::ranges::atLeast(1u) & nvxio::ranges::atMost(8u)));
    ftparser->addParameter("lk_win_size", nvxio::OptionHandler::unsignedInteger(&config.lk_win_size,
             nvxio::ranges::atLeast(3u) & nvxio::ranges::atMost(32u)));
    ftparser->addParameter("lk_num_iters", nvxio::OptionHandler::unsignedInteger(&config.lk_num_iters,
             nvxio::ranges::atLeast(1u) & nvxio::ranges::atMost(100u)));
    ftparser->addParameter("harris_k", nvxio::OptionHandler::real(&config.harris_k,
             nvxio::ranges::moreThan(0.0f)));
    ftparser->addParameter("harris_thresh", nvxio::OptionHandler::real(&config.harris_thresh,
             nvxio::ranges::moreThan(0.0f)));
    ftparser->addParameter("harris_cell_size", nvxio::OptionHandler::unsignedInteger(&config.harris_cell_size,
             nvxio::ranges::atLeast(1u)));
    ftparser->addParameter("array_capacity", nvxio::OptionHandler::unsignedInteger(&config.array_capacity,
             nvxio::ranges::atLeast(1u)));

    message = ftparser->parse(nf);

    return message.empty();
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

        std::string sourceUri = app.findSampleFilePath("cars.mp4");
        std::string configFile = app.findSampleFilePath("feature_tracker_demo_config.ini");

        app.setDescription("This demo demonstrates Feature Tracker algorithm");
        app.addOption('s', "source", "Source URI", nvxio::OptionHandler::string(&sourceUri));
        app.addOption('c', "config", "Config file path", nvxio::OptionHandler::string(&configFile));

#if defined USE_OPENCV || defined USE_GSTREAMER
        std::string maskFile;
        app.addOption('m', "mask", "Optional mask", nvxio::OptionHandler::string(&maskFile));
#endif

        app.init(argc, argv);

        //
        // Create OpenVX context
        //

        nvxio::ContextGuard context;

        //
        // Reads and checks input parameters
        //

        nvx::FeatureTracker::HarrisPyrLKParams params;
        std::string error;
        if (!read(configFile, params, error))
        {
            std::cout<<error;
            return nvxio::Application::APP_EXIT_CODE_INVALID_VALUE;
        }

        //
        // Create a Frame Source
        //

        std::unique_ptr<nvxio::FrameSource> source(
            nvxio::createDefaultFrameSource(context, sourceUri));

        if (!source || !source->open())
        {
            std::cerr << "Can't open source URI " << sourceUri << std::endl;
            return nvxio::Application::APP_EXIT_CODE_NO_RESOURCE;
        }

        if (source->getSourceType() == nvxio::FrameSource::SINGLE_IMAGE_SOURCE)
        {
            std::cerr << "Can't work on a single image." << std::endl;
            return nvxio::Application::APP_EXIT_CODE_INVALID_FORMAT;
        }

        nvxio::FrameSource::Parameters sourceParams = source->getConfiguration();

        //
        // Create a Render
        //

        std::unique_ptr<nvxio::Render> renderer(nvxio::createDefaultRender(
            context, "Feature Tracker Demo", sourceParams.frameWidth, sourceParams.frameHeight));

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

        vx_image frameExemplar = vxCreateImage(context,
            sourceParams.frameWidth, sourceParams.frameHeight, sourceParams.format);
        NVXIO_CHECK_REFERENCE(frameExemplar);
        vx_delay frame_delay = vxCreateDelay(context, (vx_reference)frameExemplar, 2);
        NVXIO_CHECK_REFERENCE(frame_delay);
        vxReleaseImage(&frameExemplar);

        vx_image prevFrame = (vx_image)vxGetReferenceFromDelay(frame_delay, -1);
        vx_image frame = (vx_image)vxGetReferenceFromDelay(frame_delay, 0);

        //
        // Load mask image if needed
        //

        vx_image mask = NULL;

#if defined USE_OPENCV || defined USE_GSTREAMER
        if (!maskFile.empty())
        {
            mask = nvxio::loadImageFromFile(context, maskFile, VX_DF_IMAGE_U8);

            vx_uint32 mask_width = 0, mask_height = 0;
            NVXIO_SAFE_CALL( vxQueryImage(mask, VX_IMAGE_ATTRIBUTE_WIDTH, &mask_width, sizeof(mask_width)) );
            NVXIO_SAFE_CALL( vxQueryImage(mask, VX_IMAGE_ATTRIBUTE_HEIGHT, &mask_height, sizeof(mask_height)) );

            if (mask_width != sourceParams.frameWidth || mask_height != sourceParams.frameHeight)
            {
                std::cerr << "The mask must have the same size as the input source." << std::endl;
                return nvxio::Application::APP_EXIT_CODE_INVALID_DIMENSIONS;
            }
        }
#endif

        //
        // Create FeatureTracker instance
        //

        std::unique_ptr<nvx::FeatureTracker> tracker(nvx::FeatureTracker::createHarrisPyrLK(context, params));

        nvxio::FrameSource::FrameStatus frameStatus;

        do
        {
            frameStatus = source->fetch(frame);
        } while (frameStatus == nvxio::FrameSource::TIMEOUT);

        if (frameStatus == nvxio::FrameSource::CLOSED)
        {
            std::cerr << "Source has no frames" << std::endl;
            return nvxio::Application::APP_EXIT_CODE_NO_FRAMESOURCE;
        }

        tracker->init(frame, mask);

        vxAgeDelay(frame_delay);

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
                frameStatus = source->fetch(frame);

                if (frameStatus == nvxio::FrameSource::TIMEOUT) {
                    continue;
                }
                if (frameStatus == nvxio::FrameSource::CLOSED) {
                    if (!source->open()) {
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

                tracker->track(frame, mask);

                proc_ms = procTimer.toc();

                //
                // Print performance results
                //

                tracker->printPerfs();
            }

            //
            // show the previous frame
            //
            renderer->putImage(prevFrame);

            //
            // Draw arrows & state
            //

            drawArrows(renderer.get(), tracker->getPrevFeatures(), tracker->getCurrFeatures());

            double total_ms = totalTimer.toc();

            std::cout << "Display Time : " << total_ms << " ms" << std::endl << std::endl;

            //
            // Add a delay to limit frame rate
            //

            app.sleepToLimitFPS(total_ms);

            total_ms = totalTimer.toc();

            totalTimer.tic();

            displayState(renderer.get(), sourceParams, proc_ms, total_ms);

            if (!renderer->flush())
            {
                eventData.shouldStop = true;
            }

            if (!eventData.pause)
            {
                vxAgeDelay(frame_delay);
            }
        }

        //
        // Release all objects
        //

        vxReleaseImage(&mask);
        vxReleaseDelay(&frame_delay);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return nvxio::Application::APP_EXIT_CODE_ERROR;
    }

    return nvxio::Application::APP_EXIT_CODE_SUCCESS;
}
