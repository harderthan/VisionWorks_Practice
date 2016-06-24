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

#ifndef OPENCVBASERENDERIMPL_HPP
#define OPENCVBASERENDERIMPL_HPP

#ifdef USE_OPENCV

#include "NVXIO/Render.hpp"
#include "NVXIO/Utility.hpp"

#include <vector>
#include <opencv2/core/core.hpp>

namespace nvxio
{

class OpenCVBaseRenderImpl : public nvxio::Render
{
public:
    OpenCVBaseRenderImpl();
    virtual void putImage(vx_image image);
    virtual void putText(const std::string& text, const TextBoxStyle& style);
    virtual void putFeatures(vx_array location, const FeatureStyle& style);
    virtual void putLines(vx_array linesGroups, const LineStyle& style);
    virtual void putConvexPoligon(vx_array verticies, const LineStyle& style);
    virtual void putMotionField(vx_image field, const MotionFieldStyle& style);
    virtual void putObjectLocation(const vx_rectangle_t& location, const DetectedObjectStyle& style);
    virtual void putCircles(vx_array circles, const CircleStyle& style);
    virtual void putArrows(vx_array old_points, vx_array new_points, const LineStyle& line_style);

    virtual vx_uint32 getWidth() const
    {
        return windowResolution.width;
    }

    virtual vx_uint32 getHeight() const
    {
        return windowResolution.height;
    }

    virtual bool resizeBuffers(vx_uint32 width, vx_uint32 height) = 0;

    virtual ~OpenCVBaseRenderImpl();

protected:

    OpenCVBaseRenderImpl(TargetType type, std::string name):
        Render(type, name)
    {}

    void composeFrame();

    struct TextBox: public TextBoxStyle
    {
        std::string text_;
        TextBox(const TextBoxStyle& style, const std::string& text):
            TextBoxStyle(style),
            text_(text)
        {}
    };

    struct FeatureGroup: public FeatureStyle
    {
        std::vector<nvx_point2f_t> features_;

        FeatureGroup(const FeatureStyle& style, vx_array features):
            FeatureStyle(style)
        {
            vx_size size;
            vx_enum item_type = 0;
            NVXIO_SAFE_CALL( vxQueryArray(features, VX_ARRAY_ATTRIBUTE_NUMITEMS, &size, sizeof(size)) );
            NVXIO_SAFE_CALL( vxQueryArray(features, VX_ARRAY_ATTRIBUTE_ITEMTYPE, &item_type, sizeof(item_type)) );
            NVXIO_ASSERT( (item_type == VX_TYPE_KEYPOINT) || (item_type == NVX_TYPE_POINT2F) ||
                          (item_type == NVX_TYPE_KEYPOINTF));

            if (size != 0)
            {
                features_.resize(size);
                vx_size stride;
                void * featureData = NULL;
                NVXIO_SAFE_CALL( vxAccessArrayRange(features, 0, size, &stride, &featureData, VX_READ_ONLY) );

                if (item_type == VX_TYPE_KEYPOINT)
                {
                    for (vx_size i = 0; i < size; i++)
                    {
                        vx_keypoint_t keypoint = vxArrayItem(vx_keypoint_t, featureData, i, stride);

                        features_[i].x = keypoint.x;
                        features_[i].y = keypoint.y;
                    }
                }
                else if (item_type == NVX_TYPE_KEYPOINTF)
                {
                    for (vx_size i = 0; i < size; i++)
                    {
                        nvx_keypointf_t keypoint = vxArrayItem(nvx_keypointf_t, featureData, i, stride);

                        features_[i].x = keypoint.x;
                        features_[i].y = keypoint.y;
                    }
                }
                else if (item_type == NVX_TYPE_POINT2F)
                {
                    for (vx_size i = 0; i < size; i++)
                    {
                        features_[i] = vxArrayItem(nvx_point2f_t, featureData, i, stride);
                    }
                }

                NVXIO_SAFE_CALL( vxCommitArrayRange(features, 0, size, featureData) );
            }
        }
    };

    struct LineGroup: public LineStyle
    {
        std::vector<nvx_point4f_t> lines_;
        LineGroup(const LineStyle& style, vx_array lines):
            LineStyle(style)
        {
            vx_size size;
            vxQueryArray(lines, VX_ARRAY_ATTRIBUTE_NUMITEMS, &size, sizeof(size));
            if (size != 0)
            {
                lines_.resize(size);

                vx_size stride;
                void* linesData = NULL;
                vxAccessArrayRange(lines, 0, size, &stride, &linesData, VX_READ_ONLY);

                for (vx_size i = 0; i < size; i++)
                {
                    lines_[i] = vxArrayItem(nvx_point4f_t, linesData, i, stride);
                }

                vxCommitArrayRange(lines, 0, size, linesData);
            }
        }
    };

    struct ArrowsGroup: public LineStyle
    {
        std::vector<nvx_point2f_t> old_points_, new_points_;

        ArrowsGroup(const LineStyle& style, vx_array old_points, vx_array new_points):
            LineStyle(style)
        {
            vx_size old_size = 0, new_size = 0;
            NVXIO_SAFE_CALL( vxQueryArray(old_points, VX_ARRAY_ATTRIBUTE_NUMITEMS, &old_size, sizeof(old_size)) );
            NVXIO_SAFE_CALL( vxQueryArray(new_points, VX_ARRAY_ATTRIBUTE_NUMITEMS, &new_size, sizeof(new_size)) );

            vx_size size = std::min(old_size, new_size);

            if (size != 0)
            {
                old_points_.resize(size);
                new_points_.resize(size);

                vx_size old_stride = 0, new_stride = 0;
                void* oldData = NULL, * newData = NULL;
                vxAccessArrayRange(old_points, 0, size, &old_stride, &oldData, VX_READ_ONLY);
                vxAccessArrayRange(new_points, 0, size, &new_stride, &newData, VX_READ_ONLY);

                for (vx_size i = 0; i < size; i++)
                {
                    old_points_[i] = vxArrayItem(nvx_point2f_t, oldData, i, old_stride);
                    new_points_[i] = vxArrayItem(nvx_point2f_t, newData, i, new_stride);
                }

                vxCommitArrayRange(old_points, 0, size, oldData);
                vxCommitArrayRange(new_points, 0, size, newData);
            }
        }
    };

    struct ConvexPoligon: public LineStyle
    {
        std::vector<vx_coordinates2d_t> verticies_;
        ConvexPoligon(const LineStyle& style, vx_array vertices):
            LineStyle(style)
        {
            vx_size size;
            vxQueryArray(vertices, VX_ARRAY_ATTRIBUTE_NUMITEMS, &size, sizeof(size));
            if (size != 0)
            {
                verticies_.resize(size);

                vx_size stride;
                void* veritciesData = NULL;
                vxAccessArrayRange(vertices, 0, size, &stride, &veritciesData, VX_READ_ONLY);

                for (vx_size i = 0; i < size; i++)
                {
                    verticies_[i] = vxArrayItem(vx_coordinates2d_t, veritciesData, i, stride);
                }

                vxCommitArrayRange(vertices, 0, size, veritciesData);
            }

        }
    };

    struct MotionField: public MotionFieldStyle
    {
        vx_image field_;
        MotionField(const MotionFieldStyle& style, vx_image field):
            MotionFieldStyle(style),
            field_(field)
        {}
    };

    struct DetectedObject: public DetectedObjectStyle
    {
        vx_rectangle_t location_;
        DetectedObject(const DetectedObjectStyle& style, const vx_rectangle_t& location):
            DetectedObjectStyle(style),
            location_(location)
        {}
    };

    struct CirclesGroup: public CircleStyle
    {
        std::vector<nvx_point3f_t> circles_;
        CirclesGroup(const CircleStyle& style, vx_array circles):
            CircleStyle(style)
        {
            vx_size num_items = 0;
            vxQueryArray(circles, VX_ARRAY_ATTRIBUTE_NUMITEMS, &num_items, sizeof(num_items));

            if (num_items != 0)
            {
                circles_.resize(num_items);

                vx_size stride;
                void* ptr = NULL;
                vxAccessArrayRange(circles, 0, num_items, &stride, &ptr, VX_READ_ONLY);

                for (vx_size i = 0; i < num_items; i++)
                {
                    circles_[i] = vxArrayItem(nvx_point3f_t, ptr, i, stride);
                }

                vxCommitArrayRange(circles, 0, num_items, ptr);
            }
        }
    };

    std::vector<TextBox> texts;
    std::vector<FeatureGroup> features;
    std::vector<LineGroup> linesGroups;
    std::vector<ArrowsGroup> arrowsGroups;
    std::vector<MotionField> motionFields;
    std::vector<DetectedObject> detectedObjects;
    std::vector<CirclesGroup> circlesGroups;
    std::vector<ConvexPoligon> poligons;

    std::vector<std::string> text_lines_tmp;

    vx_image inputFrame;
    cv::Mat displayFrame;
    std::vector<cv::Point2f> flowBuf;
    std::string windowTitle;
    cv::Size windowResolution;
    cv::Size textureResolution;
};

}

#endif // USE_OPENCV
#endif // OPENCVBASERENDERIMPL_HPP
