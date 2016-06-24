Copyright (c) 2014-2015, NVIDIA CORPORATION.  All rights reserved.

Feature Tracker Demo App
@brief Feature Tracker Demo user guide.

## Introduction ##

`nvx_demo_feature_tracker` is a simple local feature tracking demo that
uses the Harris feature detector to get the initial list of features and
tracks them using the sparse pyramidal optical flow method (Lucas-Kanade).

In the first frame, Gaussian pyramid is created and initial points are detected:

                                     (frame)
                                        |
                                 [ColorConvert]
                                        |
                    +-------------------+-------------------+
                    |                                       |
            [GaussianPyramid]                         [HarrisTrack]
                    |                                       |
            (initial pyramid)                        (initial points)

In subsequent frames, points are tracked with the Lucas-Kanade method between two pyramid images.
Then, corner detection is performed to restore lost points.

The NVIDIA extended Harris Corner function divides the image into cells
of the same size and looks for corners independently in each cell. In this way,
corners are uniformly detected across the image. These corners are the keypoints
that are tracked in the next frames.

                                        (frame)
                                           |
                                    [ColorConvert]
                                           |
                                   [ChannelExtract]
                                           |
                                   [GaussianPyramid]
                                           |
    (previous pyramid)             (current pyramid)         (points to track in the previous frame)
            |                              |                                      |
            +------------------------------+--------------------------------------+
                                           |
                                  [OpticalFlowPyrLK]
                                           |
                                   (tracked points)
                                           |
                                     [HarrisTrack]
                                           |
                         (points to track in the next frame)

@note Current implementation of Harris corner detector supports input frames only with total number of pixels
less than \f$2^{20}\f$.

`nvx_demo_feature_tracker` is installed in the following directory:

    /usr/share/visionworks/sources/demos/feature_tracker

For the steps to build sample applications, see the see: nvx_samples_and_demos_user_guides section for your OS.

## Executing the Feature Tracker Demo ##

    ./nvx_demo_feature_tracker [options]

### Command Line Options ###

This topic provides a list of supported options and the values they consume.

#### \-s, \--source ####
- Parameter: [inputUri]
- Description: Specifies the input URI. Accepted parameters include a video, an image, or an image sequence
(in .png, .jpg, .jpeg, .bmp, or .tiff format), or camera.
- Usage:

    - `--source=/path/to/video.avi` for video
    - `--source=/path/to/image.png` for image
    - `--source=/path/to/image_%04d_sequence.png` for image sequence
    - `--source=device://camera0` for the first camera
    - `--source=device://camera1` for the second camera.

@note The V4L platform has a permissions issue. The hardware decoder is used and sample must be
executed with super user permissions, i.e., with `sudo`.

#### \-c, \--config ####
- Parameter: [Config file path]
- Description: Specifies the path to the configuration file. The file contains the parameters
  of the algorithm stored in key=value format. Note that the config file contains information
  on the intrinsic parameters of the camera, so using the default config file for different
  videos may sometimes give a result with insufficient quality.

    This file contains the following parameters:

    - **pyr_levels**
        - Parameter: [integer value greater than or equal to 1 and less than or equal to 8]
        - Description: The number of levels in Gaussian pyramid. Default is 6.

    - **lk_num_iters**
        - Parameter: [integer value greater than or equal to 1 and less than or equal to 100]
        - Description: The number of iterations in the Lucas-Kanade method. Default is 5.

    - **lk_win_size**
        - Parameter: [integer value greater than or equal to 3 and less than or equal to 32]
        - Description: The window size in the Lucas-Kanade method. Default is 6.

    - **array_capacity**
        - Parameter: [integer value greater than or equal to 1]
        - Description: The capacity of points array. Default is 2000.

    - **harris_cell_size**
        - Parameter: [integer value greater than or equal to 1]
        - Description: The size of non-maximum suppression cell for corner detector. Default is 18.

    - **harris_k**
        - Parameter: [floating point value greater than zero]
        - Description: The Harris corner detector "k" parameter. Default is 0.04.

    - **harris_thresh**
        - Parameter: [floating point value greater than zero]
        - Description: The Harris corner detector threshold. Default is 2000.

- Usage:

  `./nvx_demo_feature_tracker --source=/path/to/video.avi --config=/path/to/config_file.yml`

- If the argument is omitted, the default configuration file is used.

#### \-m, \--mask ####
- Parameter: [path to image]
- Description: Specifies an optional mask to filter out features. This must be a grayscale image
that is the same size as the input source. The demo uses features only in the non-zero
regions on the mask. The parameter is available only if the demo is built with either OpenCV or GStreamer support.

#### \-h, \--help ####
- Parameter: true
- Description: Prints the help message.

### Operational Keys ###
- Use `Space` to pause/resume the demo.
- Use `ESC` to close the demo.

