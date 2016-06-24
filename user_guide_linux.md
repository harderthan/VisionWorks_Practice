Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.

Linux: Building and Running Samples and Demos
@brief Shows how to build samples and demos using native or cross compilation on Linux.

There are 2 methods you can use to build the sample applications and demos:

- [Native Build](#Native) -- Native compilation is easiest,
  but x86 desktops are faster at compilation than ARM devices,
  so if you are targeting an ARM device it might be worth cross-compiling
  from an x86 desktop instead of native-compiling on the device.

- [Cross-compilation Build](#Cross) -- Cross-compilation is faster,
  but requires some additional configuration. Cross-compilation also requires
  that you have mounted the device's file system onto your host computer. You can
  use the `sshfs` tool, NFS file system, or plug a flash card into your computer.

## Native Compilation of Sample Applications and Demos ##

Sources for all samples and demos are provided in the `libvisionworks-dev` package.
After the package installation, source code and make files are located in the
`/usr/share/visionworks/sources` directory. The directory is protected from changes,
so you need to copy its content to any directory with write access.
Here we use your home folder as an example.
All samples use `make` as a build tool.

    $ /usr/share/visionworks/sources/install-samples.sh ~/
    $ cd ~/VisionWorks-<ver>-Samples/
    $ make -j4 # add dbg=1 to make debug build

You can build an individual sample from its directory but the executable will
not be created there nor in a sub-directory. The executable is created in the same directory
as when all samples are built from the top-level directory.

## Cross-Compilation of Sample Apps and Demos for Vibrante Linux ##

You use the same source code and `make` files when cross-compiling as you do for native builds,
but you also need the cross-compiler toolchain and a mounted device file system.
All samples use `make` as the build tool, but you must provide additional options
to configure `pkg-config` and the cross-compiler.
In addition, sample applications can use OpenCV and you should make some modifications
to the OpenCV files to enable cross-compiler support, but this step is optional.
This topic describes the steps to follow to cross-compile the sample applications.

### Prerequisites

Ensure you have met the following prerequisites before cross-compiling the sample applications and demos.

- The device file system is available to be used directly from a flash card
  or mounted to the host file system via `sshfs` or NFS.
  **Note:** It is important to transform absolute symlinks to relative (e.g., use `sshfs -o transform_symlinks <src> <dst>`)
  The procedure below assumes the device file system has been mounted to `/mnt/jetson`.
- You have a Vibrante PDK toolchain installed with the same version as Vibrante installed on the device.
  If you do not, reinstall either `vibrante-vcm30t124-linux-<version>-oss-minimal-pdk.run` or
  `vibrante-t210ref-foundation-<version>-toolchain.run` package and approve toolchain installation.
  The procedure below assumes the Vibrante PDK toolchains are installed to `~/toolchains`.
- `libvisionworks` and `libvisionworks-dev` packages are installed on the device.

- You have located the sources for the sample applications in the libvisionworks-dev package at:

        /usr/share/visionworks/sources

    And copied it to a directory with write access before building. For example:

        $ /mnt/jetson/usr/share/visionworks/sources/install-samples.sh ~/
        $ cd ~/VisionWorks-<ver>-Samples/

### To cross-compile sample applications and demos

1. [Optional] If OpenCV is installed on your device, copy the `opencv.pc` file from the sample applications
   folder to the following location on the device:

        /usr/lib/pkgconfig

    You overwrite the existing file in this step.
    You must use `sudo` for this operation because root owns the target file,
    and the command must be executed on device, not the host computer.

        $ sudo cp -f /usr/share/visionworks/sources/opencv.pc /usr/lib/pkgconfig

2. Add Vibrante PDK cross compiler to your PATH environment variable:

        $ export PATH=~/toolchains/tegra-4.8.1-nv/usr/bin/armv7a-vfpv3-cortex_a15-linux-gnueabi:$PATH

    If you cross-compile for aarch64, use:

        $ export PATH=~/toolchains/tegra-4.9-nv/usr/bin/aarch64-gnu-linux:$PATH

3. Add path to Vibrante cross-compiler sysroot to the environment:

        $ export VIBRANTE_TOOLCHAIN_SYSROOT=~/toolchains/tegra-[version]-nv/usr/sysroot

4. Run the following command in the shell to change the configuration of `pkg-config` for cross-compilation:

        $ export PKG_CONFIG_SYSROOT_DIR=/mnt/jetson
        $ export PKG_CONFIG_ALLOW_SYSTEM_CFLAGS=1
        $ export PKG_CONFIG_ALLOW_SYSTEM_LIBS=1

    Add the following commands if you cross-compile for armv7:

        $ export PKG_CONFIG_PATH=$PKG_CONFIG_SYSROOT_DIR/usr/lib/arm-linux-gnueabihf/pkgconfig:$PKG_CONFIG_SYSROOT_DIR/usr/lib/pkgconfig
        $ export PKG_CONFIG_LIBDIR=$PKG_CONFIG_SYSROOT_DIR/usr/share/pkgconfig/
        $ export CC=arm-cortex_a15-linux-gnueabi-gcc
        $ export CXX=arm-cortex_a15-linux-gnueabi-g++
        $ export CPP=arm-cortex_a15-linux-gnueabi-cpp
        $ export LD=arm-cortex_a15-linux-gnueabi-ld
        $ export AR=arm-cortex_a15-linux-gnueabi-ar

    Add the following for cross-compilation for aarch64:

        $ export PKG_CONFIG_PATH=$PKG_CONFIG_SYSROOT_DIR/usr/lib/aarch64-linux-gnu/pkgconfig:$PKG_CONFIG_SYSROOT_DIR/usr/lib/pkgconfig
        $ export PKG_CONFIG_LIBDIR=$PKG_CONFIG_SYSROOT_DIR/usr/share/pkgconfig/
        $ export CC=aarch64-gnu-linux-gcc
        $ export CXX=aarch64-gnu-linux-g++
        $ export CPP=aarch64-gnu-linux-cpp
        $ export LD=aarch64-gnu-linux-ld
        $ export AR=aarch64-gnu-linux-ar

    Execute the following commands to build for ARMv7:

        $ make ARCH=armhf ARMv7=1 -j4 # add dbg=1 to make debug build

    Execute the following command to build for ARMv8:

        $ make ARCH=aarch64 ARMv8=1 -j4 # add dbg=1 to make debug build

## Running Samples and Demos ##

**Applies to:** ARM devices only. Start the X window manager:

    $ export DISPLAY=:0
    $ X -ac &
    $ blackbox $

Go to the samples directory:

    $ cd ~/VisionWorks-<ver>-Samples/sources/bin/[arch]/linux/release

Run each sample of interest by using the name of the sample. For example, to run `nvx_demo_feature_tracker`, execute:

    $ ./nvx_demo_feature_tracker

