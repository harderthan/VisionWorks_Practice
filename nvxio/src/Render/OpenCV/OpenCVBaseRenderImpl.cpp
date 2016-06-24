/*
# Copyright (c) 2014, NVIDIA CORPORATION. All rights reserved.
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

#ifdef USE_OPENCV
#include <NVXIO/Utility.hpp>

#include "Render/OpenCV/OpenCVBaseRenderImpl.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace nvxio
{

static void split(const std::string& str, char delim, std::vector<std::string>& elems)
{
    elems.clear();

    std::stringstream ss(str);
    std::string item;

    while (std::getline(ss, item, delim))
        elems.push_back(item);
}

static void drawFlowField(vx_image mf, cv::Mat& dst, std::vector<cv::Point2f>& flowBuf, cv::Scalar color, float scale_x, float scale_y, vx_uint32 gridSize = 16)
{
    vx_rectangle_t rect;
    NVXIO_SAFE_CALL( vxGetValidRegionImage(mf, &rect) );

    vx_uint8 *mv_base = NULL;
    vx_imagepatch_addressing_t mv_addr;
    NVXIO_SAFE_CALL( vxAccessImagePatch(mf, &rect, 0, &mv_addr, (void **)&mv_base, VX_READ_ONLY) );

    vx_uint32 scale = dst.cols / mv_addr.dim_x;
    gridSize /= scale;

    flowBuf.resize((dst.cols + gridSize - 1) / gridSize);
    std::fill(flowBuf.begin(), flowBuf.end(), cv::Point2f());

    for (vx_uint32 y = 0u; y < mv_addr.dim_y; y++)
    {
        cv::Point2f* mv_row = (cv::Point2f *)(mv_base + y * mv_addr.stride_y);

        for (vx_uint32 x = 0u; x < mv_addr.dim_x; x++)
        {
            flowBuf[x / gridSize].x += mv_row[x].x;
            flowBuf[x / gridSize].y += mv_row[x].y;
        }

        if ((y % gridSize) == gridSize - 1)
        {
            for (size_t x = 0; x < flowBuf.size(); ++x)
            {
                cv::Point2f v = flowBuf[x];
                v.x /= gridSize * gridSize;
                v.y /= gridSize * gridSize;

                cv::Point2f p((x * gridSize + gridSize / 2) * scale, (y - gridSize / 2) * scale);
                cv::Point2f q = p + v;

                const float hypotenuse = sqrtf(v.x * v.x + v.y * v.y) + 2.0f;
                if (hypotenuse < 3.0f || hypotenuse > 50.0f)
                    continue;

                const float angle = atan2f(-v.y, -v.x);

                // Here we lengthen the arrow by a factor of three.
                q.x = p.x - hypotenuse * cosf(angle);
                q.y = p.y - hypotenuse * sinf(angle);

                // Now we add the main line of the arrow.
                cv::line(dst, cv::Point(p.x * scale_x, p.y * scale_y), cv::Point(q.x * scale_x, q.y * scale_y), color);

                // Now add the tips of the arrow. I do some scaling so that the
                // tips look proportional to the main line of the arrow.

                const float tips_length = 9.0f * hypotenuse / 50.0f + 5.0f;

                p.x = q.x + tips_length * cosf(angle + PI_F / 6.0f);
                p.y = q.y + tips_length * sinf(angle + PI_F / 6.0f);
                cv::line(dst, cv::Point(p.x * scale_x, p.y * scale_y), cv::Point(q.x * scale_x, q.y * scale_y), color);

                p.x = q.x + tips_length * cosf(angle - PI_F / 6.0f);
                p.y = q.y + tips_length * sinf(angle - PI_F / 6.0f);
                cv::line(dst, cv::Point(p.x * scale_x, p.y * scale_y), cv::Point(q.x * scale_x, q.y * scale_y), color);
            }

            std::fill(flowBuf.begin(), flowBuf.end(), cv::Point2f());
        }
    }

    NVXIO_SAFE_CALL( vxCommitImagePatch(mf, NULL, 0, &mv_addr, mv_base) );
}

OpenCVBaseRenderImpl::OpenCVBaseRenderImpl():
    Render(nvxio::Render::UNKNOWN_RENDER, "OpenCVBaseRender")
{}

void OpenCVBaseRenderImpl::composeFrame()
{
    vx_imagepatch_addressing_t addr;
    void *result_ptr = NULL;
    vx_rectangle_t rect;
    vx_df_image format;

    rect.start_x = 0;
    rect.start_y = 0;
    NVXIO_SAFE_CALL( vxQueryImage(inputFrame, VX_IMAGE_ATTRIBUTE_WIDTH, &rect.end_x, sizeof(rect.end_x)) );
    NVXIO_SAFE_CALL( vxQueryImage(inputFrame, VX_IMAGE_ATTRIBUTE_HEIGHT, &rect.end_y, sizeof(rect.end_y)) );
    NVXIO_SAFE_CALL( vxQueryImage(inputFrame, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format)) );
    NVXIO_SAFE_CALL( vxAccessImagePatch(inputFrame, &rect, 0, &addr, &result_ptr, VX_READ_ONLY) );

    float scale_x = 1.f*windowResolution.width/addr.dim_x;
    float scale_y = 1.f*windowResolution.height/addr.dim_y;

    float ratio = std::min(scale_x, scale_y);
    cv::Size newSize(addr.dim_x*ratio, addr.dim_y*ratio);

    cv::Mat origImage;
    cv::Mat displayFrameROI(displayFrame, cv::Rect(0,0,newSize.width, newSize.height));
    switch(format)
    {
    case VX_DF_IMAGE_U8:
        origImage = cv::Mat(addr.dim_y, addr.dim_x, CV_8UC1, result_ptr, addr.stride_y);
        if (cv::Size(addr.dim_x, addr.dim_y) != windowResolution)
        {
            cv::Mat resizedImage;
            displayFrame.setTo(cv::Scalar(0,0,0,255));
            cv::resize(origImage, resizedImage, newSize);
            origImage = resizedImage;
        }
        cv::cvtColor(origImage, displayFrameROI, cv::COLOR_GRAY2RGBA);
        break;
    case VX_DF_IMAGE_RGB:
        origImage = cv::Mat(addr.dim_y, addr.dim_x, CV_8UC3, result_ptr, addr.stride_y);
        if (cv::Size(addr.dim_x, addr.dim_y) != windowResolution)
        {
            cv::Mat resizedImage;
            displayFrame.setTo(cv::Scalar(0,0,0,255));
            cv::resize(origImage, resizedImage, newSize);
            origImage = resizedImage;
        }
        cv::cvtColor(origImage, displayFrameROI, cv::COLOR_RGB2RGBA);
        break;
    case VX_DF_IMAGE_RGBX:
        origImage = cv::Mat(addr.dim_y, addr.dim_x, CV_8UC4, result_ptr, addr.stride_y);
        if (cv::Size(addr.dim_x, addr.dim_y) != windowResolution)
        {
            cv::Mat resizedImage;
            displayFrame.setTo(cv::Scalar(0,0,0,255));
            cv::resize(origImage, resizedImage, newSize);
            origImage = resizedImage;
        }
        origImage.copyTo(displayFrameROI);
        break;
    default:
        NVXIO_THROW_EXCEPTION( "Unsupported image format" );
    }

    NVXIO_SAFE_CALL( vxCommitImagePatch(inputFrame, NULL, 0, &addr, result_ptr) );

    for (const auto &feature: features)
    {
        for (vx_size i = 0; i < feature.features_.size(); i++)
        {
            cv::circle(displayFrame, cv::Point(scale_x*feature.features_[i].x, scale_y*feature.features_[i].y), 3,
                       cv::Scalar(feature.color[0], feature.color[1], feature.color[2], feature.color[3]));
        }
    }

    for (const auto &group: linesGroups)
    {
        for (vx_size i = 0; i < group.lines_.size(); i++)
        {
            cv::line(displayFrame, cv::Point(scale_x*group.lines_[i].x, scale_y*group.lines_[i].y),
                     cv::Point(scale_x*group.lines_[i].z, scale_y*group.lines_[i].w),
                     cv::Scalar(group.color[0], group.color[1], group.color[2], group.color[3]),
                     group.thickness);
        }
    }

    for (const auto &group: arrowsGroups)
    {
        for (vx_size i = 0; i < group.old_points_.size(); i++)
        {
            nvx_point2f_t p = group.old_points_[i], q = group.new_points_[i];

            nvx_point4f_t line = { p.x, p.y, q.x, q.y };

            double x_diff = (double) line.x - line.z;
            double y_diff = (double) line.y - line.w;

            const double angle = std::atan2(y_diff, x_diff);

            const double hypotenuse = std::sqrt( y_diff*y_diff + x_diff*x_diff );

            if (hypotenuse < 1.0 || hypotenuse > 50.0)
                continue;

            // Here we lengthen the arrow by a factor of three.
            line.z = (int) ((double) line.x - hypotenuse * cos(angle));
            line.w = (int) ((double) line.y - hypotenuse * sin(angle));

            // Now we add the main line of the arrow.
            cv::line(displayFrame, cv::Point(scale_x * line.x, scale_y * line.y),
                     cv::Point(scale_x * line.z, scale_y * line.w),
                     cv::Scalar(group.color[0], group.color[1], group.color[2], group.color[3]),
                     group.thickness);

            // Now add the tips of the arrow. I do some scaling so that the
            // tips look proportional to the main line of the arrow.

            const double tips_length = 9.0 * hypotenuse / 50.0 + 5.0;

            line.x = (int) ((double) line.z + tips_length * cos(angle + nvxio::PI / 6));
            line.y = (int) ((double) line.w + tips_length * sin(angle + nvxio::PI / 6));
            cv::line(displayFrame, cv::Point(scale_x * line.x, scale_y * line.y),
                     cv::Point(scale_x * line.z, scale_y * line.w),
                     cv::Scalar(group.color[0], group.color[1], group.color[2], group.color[3]),
                     group.thickness);

            line.x = (int) ((double) line.z + tips_length * cos(angle - nvxio::PI / 6));
            line.y = (int) ((double) line.w + tips_length * sin(angle - nvxio::PI / 6));
            cv::line(displayFrame, cv::Point(scale_x * line.x, scale_y * line.y),
                     cv::Point(scale_x * line.z, scale_y * line.w),
                     cv::Scalar(group.color[0], group.color[1], group.color[2], group.color[3]),
                     group.thickness);
        }
    }

    for (const auto &poligon: poligons)
    {
        vx_size vCount = poligon.verticies_.size();
        for (vx_size i = 1; i < vCount; i++)
        {
            cv::line(displayFrame, cv::Point(scale_x*poligon.verticies_[i-1].x, scale_y*poligon.verticies_[i-1].y),
                     cv::Point(scale_x*poligon.verticies_[i].x, scale_y*poligon.verticies_[i].y),
                     cv::Scalar(poligon.color[0], poligon.color[1], poligon.color[2], poligon.color[3]),
                     poligon.thickness);
        }
        cv::line(displayFrame, cv::Point(scale_x*poligon.verticies_[vCount-1].x, scale_y*poligon.verticies_[vCount-1].y),
                 cv::Point(scale_x*poligon.verticies_[0].x, scale_y*poligon.verticies_[0].y),
                 cv::Scalar(poligon.color[0], poligon.color[1], poligon.color[2], poligon.color[3]),
                 poligon.thickness);
    }

    for (const auto &object: detectedObjects)
    {
        cv::rectangle(displayFrame,
                      cv::Rect(scale_x*object.location_.start_x,
                               scale_y*object.location_.start_y,
                               scale_x*(object.location_.end_x-object.location_.start_x),
                               scale_y*(object.location_.end_y-object.location_.start_y)
                              ),
                      cv::Scalar(object.color[0],
                                 object.color[1],
                                 object.color[2],
                                 object.color[3]
                      ), object.thickness);
        if (!object.label.empty())
        {
            const int fontFace = cv::FONT_HERSHEY_PLAIN;
            const int fontScale = 1.0;
            const int fontThickness = 1;

            cv::Point org;
            org.x = scale_x*object.location_.start_x;
            org.y = scale_y*object.location_.start_y - 5;

            cv::putText(displayFrame, object.label, org,
                        fontFace, fontScale,
                        cv::Scalar(object.color[0], object.color[1], object.color[2], object.color[3]),
                        fontThickness);
        }
    }

    for (const auto &mf: motionFields)
    {
        drawFlowField(mf.field_, displayFrame, flowBuf, cv::Scalar(mf.color[0], mf.color[1], mf.color[2], mf.color[3]), scale_x, scale_y);
    }

    for (const auto &circles: circlesGroups)
    {
        for (vx_size i = 0; i < circles.circles_.size(); i++)
        {
            cv::circle(displayFrame, cv::Point(scale_x*circles.circles_[i].x, scale_y*circles.circles_[i].y),
                       std::min(scale_x, scale_y) * circles.circles_[i].z,
                       cv::Scalar(circles.color[0], circles.color[1], circles.color[2], circles.color[3]),
                       circles.thickness);
        }
    }

    for (const auto &box: texts)
    {
        const int fontFace = cv::FONT_HERSHEY_PLAIN;
        const int fontScale = 1.0;
        const int fontThickness = 1;

        split(box.text_, '\n', text_lines_tmp);

        cv::Size maxTextSize;
        for (size_t i = 0; i < text_lines_tmp.size(); ++i)
        {
            cv::Size lineSize = cv::getTextSize(text_lines_tmp[i], fontFace, fontScale, fontThickness, 0);
            maxTextSize.width = std::max(maxTextSize.width, lineSize.width);
            maxTextSize.height = std::max(maxTextSize.height, lineSize.height);
        }

        cv::Rect textBoxRoi;
        textBoxRoi.x = std::max(static_cast<int>(box.origin.x) - 5, 0);
        textBoxRoi.y = std::max(static_cast<int>(box.origin.y) - 5, 0);
        textBoxRoi.width = maxTextSize.width + 10;
        textBoxRoi.height = text_lines_tmp.size() * (maxTextSize.height + 10);

        if ((textBoxRoi.x+textBoxRoi.width <= displayFrame.cols) &&
            (textBoxRoi.y+textBoxRoi.height <= displayFrame.rows))
        {
            cv::Mat textBox(displayFrame, textBoxRoi);
            textBox.setTo(cv::Scalar(box.bgcolor[0], box.bgcolor[1], box.bgcolor[2], box.bgcolor[3]));

            for (size_t i = 0; i < text_lines_tmp.size(); ++i)
            {
                cv::Point org;
                org.x = box.origin.x;
                org.y = box.origin.y + (maxTextSize.height + 10) * (i + 1) - 10;

                cv::putText(displayFrame, text_lines_tmp[i], org,
                            fontFace, fontScale,
                            cv::Scalar(box.color[0], box.color[1], box.color[2], box.color[3]),
                            fontThickness);
            }
        }
    }

    texts.clear();
    linesGroups.clear();
    features.clear();
    motionFields.clear();
    detectedObjects.clear();
    circlesGroups.clear();
    arrowsGroups.clear();
}

void OpenCVBaseRenderImpl::putImage(vx_image image)
{
    inputFrame = image;
}

void OpenCVBaseRenderImpl::putText(const std::string &text, const TextBoxStyle &style)
{
    assert(style.bgcolor[3] == 255u); // to prevent transparent box
    assert(style.color[3] == 255u); // to prevent transparent text
    texts.push_back(TextBox(style, text));
}

void OpenCVBaseRenderImpl::putFeatures(vx_array location, const FeatureStyle &style)
{
    assert(style.color[3] == 255u); // to prevent transparent lines
    features.push_back(FeatureGroup(style, location));
}

void OpenCVBaseRenderImpl::putLines(vx_array lines, const LineStyle &style)
{
    assert(style.color[3] == 255u); // to prevent transparent lines
    linesGroups.push_back(LineGroup(style, lines));
}

void OpenCVBaseRenderImpl::putConvexPoligon(vx_array verticies, const LineStyle& style)
{
    assert(style.color[3] == 255u); // to prevent transparent lines
    poligons.push_back(ConvexPoligon(style, verticies));
}

void OpenCVBaseRenderImpl::putMotionField(vx_image field, const MotionFieldStyle &style)
{
    assert(style.color[3] == 255u); // to prevent transparent lines
    motionFields.push_back(MotionField(style, field));
}

void OpenCVBaseRenderImpl::putObjectLocation(const vx_rectangle_t &location, const DetectedObjectStyle &style)
{
    assert(style.color[3] == 255u); // to prevent transparent lines
    detectedObjects.push_back(DetectedObject(style, location));
}

void OpenCVBaseRenderImpl::putCircles(vx_array circles, const CircleStyle& style)
{
    assert(style.color[3] == 255u); // to prevent transparent circles
    circlesGroups.push_back(CirclesGroup(style, circles));
}

void OpenCVBaseRenderImpl::putArrows(vx_array old_points, vx_array new_points, const LineStyle& style)
{
    assert(style.color[3] == 255u); // to prevent transparent arrows

    arrowsGroups.push_back(ArrowsGroup(style, old_points, new_points));
}

OpenCVBaseRenderImpl::~OpenCVBaseRenderImpl()
{}

}

#endif
