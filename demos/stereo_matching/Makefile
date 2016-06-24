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

# OS info
OSUPPER := $(shell uname -s 2>/dev/null | tr "[:lower:]" "[:upper:]")
OSLOWER := $(shell uname -s 2>/dev/null | tr "[:upper:]" "[:lower:]")

OS_SIZE := $(shell getconf LONG_BIT)
OS_ARCH := $(shell uname -m | sed -e "s/i386/i686/")

# Take command line flags that override any of these settings
ifeq ($(i386),1)
	OS_SIZE := 32
	OS_ARCH := i686
endif

ifeq ($(x86_64),1)
	OS_SIZE := 64
	OS_ARCH := x86_64
endif

ifeq ($(ARMv7),1)
	OS_SIZE := 32
	OS_ARCH := armv7l
endif

ifeq ($(ARMv8),1)
        OS_SIZE := 64
        OS_ARCH := aarch64
endif

CXXFLAGS += -std=c++0x

ifneq ($(VIBRANTE_TOOLCHAIN_SYSROOT),)
	CCFLAGS += --sysroot="$(VIBRANTE_TOOLCHAIN_SYSROOT)"
endif

# Configuration-specific build flags
ifeq ($(dbg),1)
	CCFLAGS += -g
	TARGET := debug
else
	CCFLAGS += -O3 -DNDEBUG
	TARGET := release
endif

# check visionworks availability
VISION_WORKS_EXISTS := $(shell pkg-config --exists visionworks && echo "1" || echo "0")
ifeq ($(VISION_WORKS_EXISTS), 0)
$(error You must put directory containing visionworks.pc to the PKG_CONFIG_PATH environment variable)
endif

VISION_WORKS_CFLAGS := $(shell pkg-config --cflags visionworks )
VISION_WORKS_LIBS := $(shell pkg-config --libs visionworks )
EIGEN_CFLAGS := -I../../3rdparty/eigen

CUDA_CFLAGS := $(shell pkg-config --cflags cudart-7.0 cublas-7.0 cufft-7.0 nppc-7.0 nppi-7.0 npps-7.0)
CUDA_LIBS := $(shell pkg-config --libs cudart-7.0 cublas-7.0 cufft-7.0 nppc-7.0 nppi-7.0 npps-7.0)

USE_OPENCV = $(shell pkg-config --exists opencv && echo "1" || echo "0")
ifeq ($(USE_OPENCV), 1)
	OPENCV_CFLAGS := -DUSE_OPENCV=1 $(shell pkg-config --cflags opencv)
	OPENCV_LIBS := $(shell pkg-config --libs opencv)
endif

NVXIO_CFLAGS := -I../../nvxio/include -DUSE_GSTREAMER=1
NVXIO_LIBS := ../../libs/$(OS_ARCH)/$(OSLOWER)/$(TARGET)$(if $(abi),/$(abi))/libnvxio.a

INCLUDES :=
INCLUDES += $(NVXIO_CFLAGS)
INCLUDES +=  -I../../3rdparty/opengl  $(shell pkg-config --cflags glfw3)  $(shell pkg-config --cflags freetype2)
INCLUDES += $(CUDA_CFLAGS)
INCLUDES += $(OPENCV_CFLAGS)
INCLUDES += $(VISION_WORKS_CFLAGS)
INCLUDES += $(EIGEN_CFLAGS)

LIBRARIES := -L"$(PKG_CONFIG_SYSROOT_DIR)/usr/lib"

ifneq ($(VIBRANTE_TOOLCHAIN_SYSROOT),)
	LIBRARIES += -L"$(VIBRANTE_TOOLCHAIN_SYSROOT)/usr/lib"
endif

ifneq ($(PKG_CONFIG_SYSROOT_DIR),)
	ifeq ($(ARMv7),1)
		LIBRARIES += -Wl,-rpath-link="$(PKG_CONFIG_SYSROOT_DIR)/lib/arm-linux-gnueabihf"
		LIBRARIES += -Wl,-rpath-link="$(PKG_CONFIG_SYSROOT_DIR)/usr/lib"
		LIBRARIES += -Wl,-rpath-link="$(PKG_CONFIG_SYSROOT_DIR)/usr/lib/arm-linux-gnueabihf"
	endif
endif

LIBRARIES += $(NVXIO_LIBS)
LIBRARIES += $(shell pkg-config --libs freetype2) $(shell pkg-config --libs gl glfw3) $(shell pkg-config --libs xrandr xi xxf86vm x11)
LIBRARIES +=  $(shell pkg-config --libs gstreamer-base-1.0 gstreamer-pbutils-1.0 gstreamer-app-1.0)
LIBRARIES +=  
LIBRARIES += $(VISION_WORKS_LIBS)
LIBRARIES += $(CUDA_LIBS)
LIBRARIES += $(OPENCV_LIBS)

# add CUDA to runtime path
CUDA_LIB_PATH := $(subst -L,,$(shell pkg-config --libs-only-L cudart-7.0))
LDFLAGS += -Wl,--allow-shlib-undefined -pthread

# show libraries used by linker in debug mode
ifeq ($(dbg),1)
	LDFLAGS += -Wl,--trace
endif

ifeq ($(OS_SIZE),64)
	LDFLAGS += -Wl,-rpath=/usr/local/cuda-7.0/lib64
else
	LDFLAGS += -Wl,-rpath=/usr/local/cuda-7.0/lib
endif

CPP_FILES := $(wildcard *.cpp)
C_FILES := $(wildcard *.c)
OBJ_DIR := obj/$(TARGET)
OBJ_FILES_CPP := $(addprefix $(OBJ_DIR)/,$(notdir $(CPP_FILES:.cpp=.o)))
OBJ_FILES_C := $(addprefix $(OBJ_DIR)/,$(notdir $(C_FILES:.c=.o)))

OUTPUT_DIR := ../../bin/$(OS_ARCH)/$(OSLOWER)/$(TARGET)$(if $(abi),/$(abi))

################################################################################

# Target rules
all: build

build: $(OUTPUT_DIR)/nvx_demo_stereo_matching

$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

$(OBJ_DIR)/%.o: %.cpp | $(OBJ_DIR)
	$(CXX) $(INCLUDES) $(CCFLAGS) $(CXXFLAGS) -o $@ -c $<

$(OBJ_DIR)/%.o: %.c | $(OBJ_DIR)
	$(CC) $(INCLUDES) $(CCFLAGS) -o $@ -c $<

$(OUTPUT_DIR):
	mkdir -p $(OUTPUT_DIR)

$(OUTPUT_DIR)/nvx_demo_stereo_matching: $(OBJ_FILES_CPP) $(OBJ_FILES_C) $(NVXIO_LIBS) | $(OUTPUT_DIR)
	$(CXX) $(LDFLAGS) $(CCFLAGS) $(CXXFLAGS) -o $@ $^ $(LIBRARIES)

run: build
	./$(OUTPUT_DIR)/nvx_demo_stereo_matching

clean:
	rm -f $(OBJ_FILES_CPP) $(OBJ_FILES_C)
	rm -f $(OUTPUT_DIR)/nvx_demo_stereo_matching

$(NVXIO_LIBS):
	+@$(MAKE) -C ../../nvxio
