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
#include "alpha_comp_node.hpp"

//
// Define user kernel
//

// Constant for user kernel
enum {
    // Library ID
    USER_LIBRARY = 0x1,

    // Kernel ID
    USER_KERNEL_ALPHA_COMP = VX_KERNEL_BASE(VX_ID_DEFAULT, USER_LIBRARY) + 0x0,
};

// Kernel implementation
static vx_status VX_CALLBACK alphaComp_kernel(vx_node node, const vx_reference *parameters, vx_uint32 num)
{
    if (num != 6)
        return VX_FAILURE;

    vx_image src1 = (vx_image)parameters[0];
    vx_scalar s_alpha1 = (vx_scalar)parameters[1];
    vx_image src2 = (vx_image)parameters[2];
    vx_scalar s_alpha2 = (vx_scalar)parameters[3];
    vx_image dst = (vx_image)parameters[4];
    vx_scalar s_alphaOp = (vx_scalar)parameters[5];

    vx_uint8 alpha1 = 0;
    vx_uint8 alpha2 = 0;
    vx_enum alphaOp = 0;

    vx_status status = VX_SUCCESS;

    // Get scalars values

    vxReadScalarValue(s_alpha1, &alpha1);
    vxReadScalarValue(s_alpha2, &alpha2);
    vxReadScalarValue(s_alphaOp, &alphaOp);

    // Map OpenVX data objects into CUDA device memory

    vx_rectangle_t rect = {};
    vxGetValidRegionImage(src1, &rect);

    vx_uint8* src1_ptr = NULL;
    vx_imagepatch_addressing_t src1_addr;
    status = vxAccessImagePatch(src1, &rect, 0, &src1_addr, (void **)&src1_ptr, NVX_READ_ONLY_CUDA);

    if (status != VX_SUCCESS)
    {
        vxAddLogEntry((vx_reference)src1, status, "[%s:%u] Failed to access \'src1\' in AlphaComp Kernel", __FUNCTION__, __LINE__);
        return status;
    }

    vx_uint8* src2_ptr = NULL;
    vx_imagepatch_addressing_t src2_addr;
    vxAccessImagePatch(src2, &rect, 0, &src2_addr, (void **)&src2_ptr, NVX_READ_ONLY_CUDA);

    if (status != VX_SUCCESS)
    {
        vxAddLogEntry((vx_reference)src2, status, "[%s:%u] Failed to access \'src2\' in AlphaComp Kernel", __FUNCTION__, __LINE__);
        vxCommitImagePatch(src1, NULL, 0, &src1_addr, src1_ptr);
        return status;
    }

    vx_uint8* dst_ptr = NULL;
    vx_imagepatch_addressing_t dst_addr;
    status = vxAccessImagePatch(dst, &rect, 0, &dst_addr, (void **)&dst_ptr, NVX_WRITE_ONLY_CUDA);

    if (status != VX_SUCCESS)
    {
        vxAddLogEntry((vx_reference)src2, status, "[%s:%u] Failed to access \'dst\' in AlphaComp Kernel", __FUNCTION__, __LINE__);
        vxCommitImagePatch(src1, NULL, 0, &src1_addr, src1_ptr);
        vxCommitImagePatch(src2, NULL, 0, &src2_addr, src2_ptr);
        return status;
    }

    // Call NPP function

    NppiSize oSizeROI;
    oSizeROI.width = src1_addr.dim_x;
    oSizeROI.height = src1_addr.dim_y;

    NppStatus npp_status = nppiAlphaCompC_8u_C1R(src1_ptr, src1_addr.stride_y, alpha1,
                                                 src2_ptr, src2_addr.stride_y, alpha2,
                                                 dst_ptr, dst_addr.stride_y,
                                                 oSizeROI,
                                                 static_cast<NppiAlphaOp>(alphaOp));
    if (npp_status != NPP_SUCCESS)
    {
        vxAddLogEntry((vx_reference)node, VX_FAILURE, "[%s:%u] nppiAlphaCompC_8u_C1R error", __FUNCTION__, __LINE__);
        status = VX_FAILURE;
    }

    cudaError_t cuda_err = cudaDeviceSynchronize();
    if (cuda_err != cudaSuccess)
    {
        vxAddLogEntry((vx_reference)node, VX_FAILURE, "[%s:%u] CUDA error : %s", __FUNCTION__, __LINE__, cudaGetErrorString(cuda_err));
        status = VX_FAILURE;
    }

    // Unmap OpenVX data objects from CUDA device memory

    vxCommitImagePatch(src1, &rect, 0, &src1_addr, src1_ptr);
    vxCommitImagePatch(src2, &rect, 0, &src2_addr, src2_ptr);
    vxCommitImagePatch(dst, &rect, 0, &dst_addr, dst_ptr);

    return status;
}

// Input validator
static vx_status VX_CALLBACK alphaComp_input_validate(vx_node node, vx_uint32 index)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;

    if (index == 0)
    {
        vx_parameter param0 = vxGetParameterByIndex(node, 0);

        vx_image src1 = NULL;
        vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &src1, sizeof(src1));

        vx_df_image format = 0;
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

        if (format == VX_DF_IMAGE_U8)
        {
            status = VX_SUCCESS;
        }
        else
        {
            vxAddLogEntry((vx_reference)src1, status, "[%s:%u] Invalid format for \'src1\' in AlphaComp Kernel, it should be VX_DF_IMAGE_U8", __FUNCTION__, __LINE__);
        }

        vxReleaseImage(&src1);
        vxReleaseParameter(&param0);
    }
    else if (index == 1)
    {
        vx_parameter param1 = vxGetParameterByIndex(node, 1);

        vx_scalar alpha1 = NULL;
        vxQueryParameter(param1, VX_PARAMETER_ATTRIBUTE_REF, &alpha1, sizeof(alpha1));

        vx_enum type = 0;
        vxQueryScalar(alpha1, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));

        if (type == VX_TYPE_UINT8)
        {
            status = VX_SUCCESS;
        }
        else
        {
            vxAddLogEntry((vx_reference)alpha1, status, "[%s:%u] Invalid format for \'alpha1\' in AlphaComp Kernel, it should be VX_TYPE_UINT8", __FUNCTION__, __LINE__);
        }

        vxReleaseScalar(&alpha1);
        vxReleaseParameter(&param1);
    }
    else if (index == 2)
    {
        vx_parameter param0 = vxGetParameterByIndex(node, 0);
        vx_parameter param2 = vxGetParameterByIndex(node, 2);

        vx_image src1 = NULL;
        vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &src1, sizeof(src1));

        vx_image src2 = NULL;
        vxQueryParameter(param2, VX_PARAMETER_ATTRIBUTE_REF, &src2, sizeof(src2));

        vx_uint32 src1_width = 0, src1_height = 0;
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_WIDTH, &src1_width, sizeof(src1_width));
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_HEIGHT, &src1_height, sizeof(src1_height));

        vx_df_image src1_format = 0;
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_FORMAT, &src1_format, sizeof(src1_format));

        vx_uint32 src2_width = 0, src2_height = 0;
        vxQueryImage(src2, VX_IMAGE_ATTRIBUTE_WIDTH, &src2_width, sizeof(src2_width));
        vxQueryImage(src2, VX_IMAGE_ATTRIBUTE_HEIGHT, &src2_height, sizeof(src2_height));

        vx_df_image src2_format = 0;
        vxQueryImage(src2, VX_IMAGE_ATTRIBUTE_FORMAT, &src2_format, sizeof(src2_format));

        if (src2_format == src1_format && src2_height == src1_height && src2_width == src1_width)
        {
            status = VX_SUCCESS;
        }
        else
        {
            vxAddLogEntry((vx_reference)src2, status, "[%s:%u] \'src1\' and \'src2\' have different size/format in AlphaComp Kernel", __FUNCTION__, __LINE__);
        }

        vxReleaseImage(&src1);
        vxReleaseImage(&src2);
        vxReleaseParameter(&param0);
        vxReleaseParameter(&param2);
    }
    else if (index == 3)
    {
        vx_parameter param3 = vxGetParameterByIndex(node, 3);

        vx_scalar alpha2 = NULL;
        vxQueryParameter(param3, VX_PARAMETER_ATTRIBUTE_REF, &alpha2, sizeof(alpha2));

        vx_enum type = 0;
        vxQueryScalar(alpha2, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));

        if (type == VX_TYPE_UINT8)
        {
            status = VX_SUCCESS;
        }
        else
        {
            vxAddLogEntry((vx_reference)alpha2, status, "[%s:%u] Invalid format for \'alpha2\' in AlphaComp Kernel, it should be VX_TYPE_UINT8", __FUNCTION__, __LINE__);
        }

        vxReleaseScalar(&alpha2);
        vxReleaseParameter(&param3);
    }
    else if (index == 5)
    {
        vx_parameter param5 = vxGetParameterByIndex(node, 5);

        vx_scalar alphaOp = NULL;
        vxQueryParameter(param5, VX_PARAMETER_ATTRIBUTE_REF, &alphaOp, sizeof(alphaOp));

        vx_enum type = 0;
        vxQueryScalar(alphaOp, VX_SCALAR_ATTRIBUTE_TYPE, &type, sizeof(type));

        if (type == VX_TYPE_ENUM)
        {
            status = VX_SUCCESS;
        }
        else
        {
            vxAddLogEntry((vx_reference)alphaOp, status, "[%s:%u] Invalid format for \'alphaOp\' in AlphaComp Kernel, it should be VX_TYPE_ENUM", __FUNCTION__, __LINE__);
        }

        vxReleaseScalar(&alphaOp);
        vxReleaseParameter(&param5);
    }

    return status;
}

// Output validator
static vx_status VX_CALLBACK alphaComp_output_validate(vx_node node, vx_uint32 index, vx_meta_format meta)
{
    vx_status status = VX_ERROR_INVALID_PARAMETERS;

    if (index == 4)
    {
        vx_parameter param0 = vxGetParameterByIndex(node, 0);

        vx_image src1 = NULL;
        vxQueryParameter(param0, VX_PARAMETER_ATTRIBUTE_REF, &src1, sizeof(src1));

        vx_uint32 width = 0, height = 0;
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

        vx_df_image format = 0;
        vxQueryImage(src1, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));

        vxSetMetaFormatAttribute(meta, VX_IMAGE_ATTRIBUTE_FORMAT, &format, sizeof(format));
        vxSetMetaFormatAttribute(meta, VX_IMAGE_ATTRIBUTE_WIDTH, &width, sizeof(width));
        vxSetMetaFormatAttribute(meta, VX_IMAGE_ATTRIBUTE_HEIGHT, &height, sizeof(height));

        vxReleaseImage(&src1);
        vxReleaseParameter(&param0);

        status = VX_SUCCESS;
    }

    return status;
}

// Register user defined kernel in OpenVX context
vx_status registerAlphaCompKernel(vx_context context)
{
    vx_status status = VX_SUCCESS;

    // Use `gpu:` prefix to notify the framework, that the kernel uses GPU implementation

    vx_kernel kernel = vxAddKernel(context, const_cast<vx_char*>("gpu:user.kernel.alpha_comp"), USER_KERNEL_ALPHA_COMP,
                                   alphaComp_kernel,
                                   6,    // numParams
                                   alphaComp_input_validate,
                                   alphaComp_output_validate,
                                   NULL, // init
                                   NULL  // deinit
                                   );

    status = vxGetStatus((vx_reference)kernel);
    if (status != VX_SUCCESS)
    {
        vxAddLogEntry((vx_reference)context, status, "Failed to create AlphaComp Kernel");
        return status;
    }

    status |= vxAddParameterToKernel(kernel, 0, VX_INPUT , VX_TYPE_IMAGE , VX_PARAMETER_STATE_REQUIRED); // src1
    status |= vxAddParameterToKernel(kernel, 1, VX_INPUT , VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED); // alpha1
    status |= vxAddParameterToKernel(kernel, 2, VX_INPUT , VX_TYPE_IMAGE , VX_PARAMETER_STATE_REQUIRED); // src2
    status |= vxAddParameterToKernel(kernel, 3, VX_INPUT , VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED); // alpha2
    status |= vxAddParameterToKernel(kernel, 4, VX_OUTPUT, VX_TYPE_IMAGE , VX_PARAMETER_STATE_REQUIRED); // dst
    status |= vxAddParameterToKernel(kernel, 5, VX_INPUT , VX_TYPE_SCALAR, VX_PARAMETER_STATE_REQUIRED); // alphaOp

    if (status != VX_SUCCESS)
    {
        vxReleaseKernel(&kernel);
        vxAddLogEntry((vx_reference)context, status, "Failed to initialize AlphaComp Kernel parameters");
        return VX_FAILURE;
    }

    status = vxFinalizeKernel(kernel);

    if (status != VX_SUCCESS)
    {
        vxReleaseKernel(&kernel);
        vxAddLogEntry((vx_reference)context, status, "Failed to finalize AlphaComp Kernel");
        return VX_FAILURE;
    }

    return status;
}

// Create AlphaComp node
vx_node alphaCompNode(vx_graph graph, vx_image src1, vx_scalar alpha1, vx_image src2, vx_scalar alpha2, vx_image dst, vx_scalar alphaOp)
{
    vx_node node = NULL;

    vx_kernel kernel = vxGetKernelByEnum(vxGetContext((vx_reference)graph), USER_KERNEL_ALPHA_COMP);

    if (vxGetStatus((vx_reference)kernel) == VX_SUCCESS)
    {
        node = vxCreateGenericNode(graph, kernel);
        vxReleaseKernel(&kernel);

        if (vxGetStatus((vx_reference)node) == VX_SUCCESS)
        {
            vxSetParameterByIndex(node, 0, (vx_reference)src1);
            vxSetParameterByIndex(node, 1, (vx_reference)alpha1);
            vxSetParameterByIndex(node, 2, (vx_reference)src2);
            vxSetParameterByIndex(node, 3, (vx_reference)alpha2);
            vxSetParameterByIndex(node, 4, (vx_reference)dst);
            vxSetParameterByIndex(node, 5, (vx_reference)alphaOp);
        }
    }

    return node;
}

#endif // USE_OPENCV
