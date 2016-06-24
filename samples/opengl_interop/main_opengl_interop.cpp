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

#ifdef _WIN32
// cuda_gl_interop.h includes GL/gl.h, which requires Windows.h to work.
#define NOMINMAX
#include <Windows.h>
#endif

#include <stdio.h>
#include <memory>
#include <iostream>
#include <iomanip>

#include <VX/vx.h>
#include <NVX/nvx_timer.hpp>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

// Should be included before cuda_gl_interop.h
#include "NVXIO/OpenGL.hpp"

#include <cuda_runtime.h>
#include <cuda_gl_interop.h>

#include "NVXIO/FrameSource.hpp"
#include "NVXIO/Application.hpp"
#include "NVXIO/Utility.hpp"

//
// OpenGL Window
//

class Window
{
public:
    Window(const char* title, int width, int height);
    ~Window();

    // Make the context of the specified window current for the calling thread.
    void makeCurrentContext();

    // Swap the front and back buffers of the specified window
    // and processe all pending events.
    void refresh();

    bool isAlive() const;
    void close();

    GLint getWidth() const { return width_; }
    GLint getHeight() const { return height_; }

private:
    class GLFWInit
    {
    public:
        GLFWInit();
        ~GLFWInit();

    private:
        static void glfwErrorCallback(int error, const char* description);
    };

    static void glfwKeyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    GLFWwindow* window_;
    GLint width_, height_;
};

Window::GLFWInit::GLFWInit()
{
    glfwSetErrorCallback(glfwErrorCallback);
    if (!glfwInit())
    {
        NVXIO_THROW_EXCEPTION("GLFW Error : Failed to initialize");
    }
}

Window::GLFWInit::~GLFWInit()
{
    glfwTerminate();
}

void Window::GLFWInit::glfwErrorCallback(int /*error*/, const char* description)
{
    std::cerr << "GLFW Error : " << description << std::endl;
}

Window::Window(const char* title, int width, int height) :
    window_(nullptr)
{
    static GLFWInit initGLFW;
    (void) initGLFW;

    // Make window size fixed
    glfwWindowHint(GLFW_RESIZABLE, 0);

    // Initialize GLES context if needed
#ifdef USE_GLES
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);
#else
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    int count = 0;
    GLFWmonitor ** monitors = glfwGetMonitors(&count);

    if (count == 0)
    {
        NVXIO_THROW_EXCEPTION("Glfw: no monitors found\n");
    }

    int maxPixels = 0;
    const GLFWvidmode* mode = NULL;

    for (int i = 0; i < count; ++i)
    {
        const GLFWvidmode* currentMode = glfwGetVideoMode(monitors[i]);
        int currentPixels = currentMode->width * currentMode->height;

        if (maxPixels < currentPixels)
        {
            mode = currentMode;
            maxPixels = currentPixels;
        }
    }

    vx_uint32 wndWidth = width, wndHeight = height;

    if ((width <= mode->width) && (height <= mode->height))
    {
        wndWidth = width;
        wndHeight = height;
    }
    else
    {
        float widthRatio = static_cast<float>(mode->width) / width;
        float heightRatio = static_cast<float>(mode->height) / height;
        vx_float32 scaleRatio = std::min(widthRatio, heightRatio);

        wndWidth = static_cast<vx_uint32>(scaleRatio * width);
        wndHeight = static_cast<vx_uint32>(scaleRatio * height);
    }

    int xpos = (mode->width - wndWidth) >> 1;
    int ypos = (mode->height - wndHeight) >> 1;

    window_ = glfwCreateWindow(wndWidth, wndHeight, title, nullptr, nullptr);
    if (!window_)
    {
        NVXIO_THROW_EXCEPTION("GLFW Error : Failed to create window");
    }

    glfwGetFramebufferSize(window_, &width_, &height_);

    if ((width_ != width) || (height_ != height))
    {
        printf("WARNING: the created window has [%d x %d], but [%d x %d] is specified\n",
               width_, height_, width, height);
    }

    glfwSetWindowPos(window_, xpos, ypos);
    glfwSetKeyCallback(window_, glfwKeyCallback);
}

Window::~Window()
{
    glfwDestroyWindow(window_);
}

void Window::makeCurrentContext()
{
    glfwMakeContextCurrent(window_);
}

void Window::refresh()
{
    glfwSwapBuffers(window_);
    glfwPollEvents();
}

bool Window::isAlive() const
{
    return !glfwWindowShouldClose(window_);
}

void Window::close()
{
    glfwSetWindowShouldClose(window_, 1);
}

void Window::glfwKeyCallback(GLFWwindow* window, int key, int /*scancode*/,
                             int action, int /*mods*/)
{
    if ((key == GLFW_KEY_ESCAPE) && (action == GLFW_PRESS))
    {
        glfwSetWindowShouldClose(window, 1);
    }
}

//
// OpenGL Render
//

#define NVXIO_CUDA_SAFE_CALL(cudaOp) \
    do \
    { \
        cudaError_t err = (cudaOp); \
        if (err != cudaSuccess) \
        { \
            NVXIO_THROW_EXCEPTION( \
                "CUDA Error in " << # cudaOp \
                << __FILE__ << " file " \
                << __LINE__ << " line : " \
                << cudaGetErrorString(err)); \
        } \
    } while (0)

static const GLchar* vertShaderSource[] =
{
#ifdef USE_GLES
    "#version 310 es\n",
    "precision mediump float;\n",
#else
    "#version 430\n",
#endif
    "in layout (location = 0) vec2 Position;\n",
    "in layout (location = 1) vec2 TexCoord;\n",
    "\n",
    "out vec2 TexCoord0;\n",
    "\n"
    "void main()\n",
    "{\n",
        "gl_Position = vec4(Position, 0.0f, 1.0f);\n",
        "TexCoord0 = TexCoord;\n",
    "}\n"
};

static const GLchar* fragShaderSource[] =
{
#ifdef USE_GLES
    "#version 310 es\n",
    "precision mediump float;\n",
#else
    "#version 430\n",
#endif
    "in vec2 TexCoord0;\n",
    "out vec4 FragColor;\n",
    "\n",
    "uniform layout (location = 0) vec2 Scale;\n",
    "\n",
    "uniform layout (binding = 0) sampler2D gSampler;\n",
    "\n",
    "void main()\n",
    "{\n",
    "   vec2 coord = TexCoord0 * Scale;",
    "   if (coord.x >= 1.0f || coord.y >= 1.0f)"
    "       FragColor = vec4(0.0f, 0.0f, 0.0f, 1.0f);"
    "   else"
    "       FragColor = texture(gSampler, coord);\n",
    "}\n"
};

class Render
{
public:
    Render(GLuint imageWidth, GLuint imageHeight,
           GLuint windowWidth, GLuint windowHeigh);
    ~Render();

    void render(vx_image image);

private:
    void createTexture();
    void createVertexBuffer();
    void createShaderProgram();
    void addShader(GLenum shaderType, const GLchar* sources[], GLsizei count);
    void createCudaGraphicsResource();

    void updateTexture(vx_image image);
    void renderTexture();

    nvxio::GLFunctions gl_;

    GLuint tex_;
    GLuint vbo_;
    GLuint vao_;
    GLuint shaderProgram_;

    cudaGraphicsResource_t res_;
    cudaStream_t stream_;

    GLuint imageWidth_, imageHeight_;
    GLfloat scaleX_, scaleY_;
};

Render::Render(GLuint imageWidth, GLuint imageHeight,
               GLuint windowWidth, GLuint windowHeight) :
    gl_{}, tex_(0), vbo_(0), vao_(0), shaderProgram_(0),
    res_(nullptr), stream_(nullptr),
    imageWidth_(imageWidth), imageHeight_(imageHeight),
    scaleX_(1.0f), scaleY_(1.0f)
{
    // Load OpenGL functions
    nvxio::loadGLFunctions(&gl_);

    createTexture();
    createShaderProgram();
    createVertexBuffer();
    createCudaGraphicsResource();

    scaleX_ = windowWidth / static_cast<GLfloat>(imageWidth_);
    scaleY_ = windowHeight / static_cast<GLfloat>(imageHeight_);

    GLfloat scale = std::min(scaleX_, scaleY_);
    scaleX_ /= scale;
    scaleY_ /= scale;
}

Render::~Render()
{
    cudaStreamDestroy(stream_);
    cudaGraphicsUnregisterResource(res_);

    gl_.DeleteProgram(shaderProgram_);
    gl_.DeleteBuffers(1, &vbo_);
    gl_.DeleteVertexArrays(1, &vao_);
    gl_.DeleteTextures(1, &tex_);
}

void Render::render(vx_image image)
{
    updateTexture(image);
    renderTexture();
}

void Render::createTexture()
{
    NVXIO_ASSERT(tex_ == 0);

    gl_.GenTextures(1, &tex_);
    gl_.ActiveTexture(GL_TEXTURE0);
    gl_.BindTexture(GL_TEXTURE_2D, tex_);
    gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    gl_.TexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, imageWidth_, imageHeight_);
    gl_.BindTexture(GL_TEXTURE_2D, 0);
}

void Render::createVertexBuffer()
{
    const GLfloat vertices[] =
    {
        /* vertex coords */     /* tex coords */
        -1.0f, -1.0f,        0.0f, 1.0f,
         1.0f, -1.0f,        1.0f, 1.0f,
        -1.0f,  1.0f,        0.0f, 0.0f,
         1.0f,  1.0f,        1.0f, 0.0f
    };

    gl_.GenVertexArrays(1, &vao_);
    gl_.BindVertexArray(vao_);

    gl_.GenBuffers(1, &vbo_);
    gl_.BindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_.BufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    gl_.EnableVertexAttribArray(0);
    gl_.VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                            nullptr);

    gl_.EnableVertexAttribArray(1);
    gl_.VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(GLfloat),
                            reinterpret_cast<const GLvoid*>(2 * sizeof(GLfloat)));

    gl_.BindBuffer(GL_ARRAY_BUFFER, 0);
    gl_.BindVertexArray(0);
}

void Render::createShaderProgram()
{
    shaderProgram_ = gl_.CreateProgram();
    if (shaderProgram_ == 0)
    {
        NVXIO_THROW_EXCEPTION(
            "OpenGL Error : Failed to create shader program");
    }

    addShader(GL_VERTEX_SHADER, vertShaderSource, static_cast<GLsizei>(nvxio::dimOf(vertShaderSource)));
    addShader(GL_FRAGMENT_SHADER, fragShaderSource, static_cast<GLsizei>(nvxio::dimOf(fragShaderSource)));

    gl_.LinkProgram(shaderProgram_);

    GLint status = 0;
    GLchar infoLog[1024] = { 0 };

    gl_.GetProgramiv(shaderProgram_, GL_LINK_STATUS, &status);
    if (status == 0)
    {
        gl_.GetProgramInfoLog(shaderProgram_, sizeof(infoLog), NULL, infoLog);
        NVXIO_THROW_EXCEPTION(
            "OpenGL Error : Failed to link shader program :" << infoLog);
    }

    gl_.ValidateProgram(shaderProgram_);

    gl_.GetProgramiv(shaderProgram_, GL_VALIDATE_STATUS, &status);
    if (!status)
    {
        gl_.GetProgramInfoLog(shaderProgram_, sizeof(infoLog), NULL, infoLog);
        NVXIO_THROW_EXCEPTION(
            "OpenGL Error : Invalid shader program :" << infoLog);
    }
}

void Render::addShader(GLenum shaderType, const GLchar* sources[], GLsizei count)
{
    const char* shaderTypeStr = (shaderType == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT");

    GLuint shaderObj = gl_.CreateShader(shaderType);
    if (shaderObj == 0)
    {
        NVXIO_THROW_EXCEPTION(
            "OpenGL Error : Failed to create " << shaderTypeStr << " shader");
    }

    gl_.ShaderSource(shaderObj, count, sources, NULL);

    gl_.CompileShader(shaderObj);

    GLint status = 0;
    gl_.GetShaderiv(shaderObj, GL_COMPILE_STATUS, &status);
    if (status == 0)
    {
        GLchar infoLog[1024] = { 0 };
        gl_.GetShaderInfoLog(shaderObj, sizeof(infoLog), NULL, infoLog);

        NVXIO_THROW_EXCEPTION(
            "OpenGL Error : Failed to compile " << shaderTypeStr << " shader : " << infoLog);
    }

    gl_.AttachShader(shaderProgram_, shaderObj);

    gl_.DeleteShader(shaderObj);
}

void Render::createCudaGraphicsResource()
{
    NVXIO_ASSERT(tex_ != 0);

    NVXIO_CUDA_SAFE_CALL( cudaGraphicsGLRegisterImage(&res_, tex_, GL_TEXTURE_2D,
                                                      cudaGraphicsRegisterFlagsWriteDiscard | cudaGraphicsRegisterFlagsSurfaceLoadStore) );

    NVXIO_CUDA_SAFE_CALL( cudaStreamCreate(&stream_) );
}

void Render::updateTexture(vx_image image)
{
    vx_df_image format = 0;
    NVXIO_SAFE_CALL( vxQueryImage(image, VX_IMAGE_ATTRIBUTE_FORMAT,
                                  &format, sizeof(format)) );
    NVXIO_ASSERT( format == VX_DF_IMAGE_RGBX );

    vx_rectangle_t rect = { 0u, 0u, imageWidth_, imageHeight_};
    vx_imagepatch_addressing_t addr;
    void *dev_ptr = NULL;
    NVXIO_SAFE_CALL( vxAccessImagePatch(image, &rect, 0, &addr, &dev_ptr,
                                        NVX_READ_ONLY_CUDA) );

    NVXIO_ASSERT( addr.dim_x == imageWidth_ );
    NVXIO_ASSERT( addr.dim_y == imageHeight_ );

    NVXIO_CUDA_SAFE_CALL( cudaGraphicsMapResources(1, &res_, stream_) );

    cudaArray_t cudaArr = NULL;
    NVXIO_CUDA_SAFE_CALL( cudaGraphicsSubResourceGetMappedArray(&cudaArr, res_, 0, 0) );

    NVXIO_CUDA_SAFE_CALL( cudaMemcpy2DToArrayAsync(cudaArr, 0, 0,
                                                   dev_ptr, addr.stride_y, addr.dim_x * 4 * sizeof(GLubyte), addr.dim_y,
                                                   cudaMemcpyDeviceToDevice,
                                                   stream_) );

    NVXIO_CUDA_SAFE_CALL( cudaGraphicsUnmapResources(1, &res_, stream_) );

    NVXIO_CUDA_SAFE_CALL( cudaStreamSynchronize(stream_) );

    NVXIO_SAFE_CALL( vxCommitImagePatch(image, NULL, 0, &addr, dev_ptr) );
}

void Render::renderTexture()
{
    gl_.UseProgram(shaderProgram_);
    gl_.BindVertexArray(vao_);

    gl_.ActiveTexture(GL_TEXTURE0);
    gl_.BindTexture(GL_TEXTURE_2D, tex_);

    gl_.Uniform2f(0, scaleX_, scaleY_);

    gl_.DrawArrays(GL_TRIANGLE_STRIP, 0, 4);

    gl_.UseProgram(0);
    gl_.BindVertexArray(0);
    gl_.BindTexture(GL_TEXTURE_2D, 0);
}

//
// main - Application entry point
//

int main(int argc, char** argv)
{
    try
    {
        nvxio::Application &app = nvxio::Application::get();

        //
        // Parse command line arguments
        //

        std::string input = app.findSampleFilePath("cars.mp4");

        app.setDescription("This sample plays a video from video-file or camera using OpenGL");
        app.addOption('s', "source", "Input URI", nvxio::OptionHandler::string(&input));
        app.init(argc, argv);

        //
        // Create OpenVX context
        //

        nvxio::ContextGuard context;

        //
        // Create FrameSource object
        //

        std::unique_ptr<nvxio::FrameSource> source(nvxio::createDefaultFrameSource(context, input));
        if (!source || !source->open())
        {
            std::cerr << "FrameSource Error : Cannot open source " << input << std::endl;
            return nvxio::Application::APP_EXIT_CODE_NO_RESOURCE;
        }

        nvxio::FrameSource::Parameters config = source->getConfiguration();
        if (config.format != VX_DF_IMAGE_RGBX)
        {
            char fourChar[5] =
            {
                static_cast<char>((config.format >> 24) & 0xFF),
                static_cast<char>((config.format >> 16) & 0xFF),
                static_cast<char>((config.format >> 8) & 0xFF),
                static_cast<char>(config.format & 0xFF),
                0
            };

            std::cerr << "FrameSource Error : unsupported input format " << fourChar << std::endl;
            return nvxio::Application::APP_EXIT_CODE_INVALID_FORMAT;
        }

        vx_image frame = vxCreateImage(context, config.frameWidth,
                                       config.frameHeight, config.format);
        NVXIO_CHECK_REFERENCE(frame);

        //
        // Render loop
        //

        Window wnd("OpenGL Interop Sample", config.frameWidth, config.frameHeight);
        wnd.makeCurrentContext();

        Render render(config.frameWidth, config.frameHeight,
                      wnd.getWidth(), wnd.getHeight());

        nvx::Timer timeStampTimer;
        timeStampTimer.tic();

        while (wnd.isAlive())
        {
            // Fetch new frames directly to CUDA memory

            nvxio::FrameSource::FrameStatus frameStatus = source->fetch(frame);

            if (frameStatus == nvxio::FrameSource::TIMEOUT)
            {
                // Do nothing
            }
            else if (frameStatus == nvxio::FrameSource::CLOSED)
            {
                // Close the sample

                break;
            }
            else // nvxio::FrameSource::OK
            {
                // Render frame

                render.render(frame);

                // Measure performance

                double time = timeStampTimer.toc();
                timeStampTimer.tic();

                std::cout << "NO PROCESSING" << std::endl;
                std::cout << "Graph Time : " << std::fixed << std::setprecision(1) << 0 << " ms" << std::endl;
                std::cout << "FRAME RATE IS NOT CONSTRAINED" << std::endl;
                std::cout << "Display Time : " << std::fixed << std::setprecision(1) << time << " ms" << std::endl;
                std::cout << "Esc - close the sample" << std::endl;
                std::cout << std::endl;
            }

            // Swap OpenGL buffers and process window events

            wnd.refresh();
        }

        //
        // Release all objects
        //

        source->close();
        vxReleaseImage(&frame);

        return nvxio::Application::APP_EXIT_CODE_SUCCESS;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
        return nvxio::Application::APP_EXIT_CODE_ERROR;
    }
}
