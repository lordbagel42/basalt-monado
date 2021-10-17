# Basalt for Monado

This is a fork of [Basalt](https://gitlab.com/VladyslavUsenko/basalt) with some
modifications so that it can be used from Monado for SLAM tracking. Many thanks
to the Basalt authors.

## Index

- [Basalt for Monado](#basalt-for-monado)
  - [Index](#index)
  - [Installation](#installation)
    - [Build and Install Directories](#build-and-install-directories)
    - [Dependencies](#dependencies)
    - [Build Basalt](#build-basalt)
    - [Monado Specifics](#monado-specifics)
  - [Using a RealSense Camera](#using-a-realsense-camera)
    - [Overview of the Setup (D455)](#overview-of-the-setup-d455)
      - [SLAM-Tracked RealSense Driver](#slam-tracked-realsense-driver)
      - [RealSense-Tracked Qwerty Driver](#realsense-tracked-qwerty-driver)
    - [Non-D455 RealSense Devices](#non-d455-realsense-devices)
      - [Configuring the RealSense Pipeline](#configuring-the-realsense-pipeline)
      - [Configuring Basalt](#configuring-basalt)
  - [Notes on Basalt Usage](#notes-on-basalt-usage)

## Installation

This was tested on both Ubuntu 20.04 and 18.04, be sure to open an issue if the
steps don't work for you.

### Build and Install Directories

To not clutter your system directories, let's set two environment variables,
`$bsltdeps` and `$bsltinstall` that point to existing empty build and install
directories respectively. These directories will contain everything produced in
this guide besides installed apt dependencies.

```bash
# Change the paths accordingly
export bsltinstall=/home/mateo/Documents/apps/bsltinstall
export bsltdeps=/home/mateo/Documents/apps/bsltdeps
```

Let's extend our system paths with those.

```bash
export PATH=$bsltinstall/bin:$PATH
export PKG_CONFIG_PATH=$bsltinstall/lib/pkgconfig:$PKG_CONFIG_PATH # for compile time pkg-config
export LD_LIBRARY_PATH=$bsltinstall/lib/:$LD_LIBRARY_PATH # for runtime ld
export LIBRARY_PATH=$bsltinstall/lib/:$LIBRARY_PATH # for compile time gcc
```

### Dependencies

<!-- TODO@mateosss: If and when I update basalt, the required eigen version is
3.4.0, for that is just better to require the user to always install eigen -->

Most dependencies will be automatically set by basalt, however for Eigen there
is a catch if you are using a distro which packages a version older than 3.3.7.
This is not a problem on Ubuntu 20.04 but for 18.04 you will need to install a
newer Eigen version as follows:

```bash
cd $bsltdeps
wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip
cd eigen-3.4.0 && mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall
make install
```

### Build Basalt

```bash
cd $bsltdeps
git clone --recursive git@gitlab.freedesktop.org:mateosss/basalt.git
./basalt/scripts/install_deps.sh
sed -i "s#/home/mateo/Documents/apps/bsltdeps/#$bsltdeps/#" ../data/monado/*.toml
cd basalt && mkdir build && cd build
# if you didn't need to do a custom Eigen installation, set -DEIGEN_ROOT=/usr/include/eigen3 instead
cmake .. -DCMAKE_INSTALL_PREFIX=$bsltinstall -DCMAKE_BUILD_TYPE=RelWithDebInfo -DEIGEN_ROOT=$bsltinstall/include/eigen3
make install -j12
```

### Monado Specifics

You'll need to compile Monado with the same Eigen as Basalt, so if you used a
custom Eigen installation, build with
`-DEIGEN3_INCLUDE_DIR=$bsltinstall/include/eigen3` otherwise Monado will
automatically use your system's Eigen. Additionally, set `export CFLAGS=-march=native CXXFLAGS=-march=native` before compiling (because we've
been building everything with `-march=native` until this point and not doing so
might result in weird crashes related to Eigen).

Run an OpenXR app like `hello_xr` with the following environment variables set

```bash
export EUROC_PATH=/path/to/euroc/V1_01_easy/ # Set euroc dataset path. You can get a dataset from http://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/vicon_room1/V1_01_easy/V1_01_easy.zip
export EUROC_LOG=debug
export EUROC_HMD=false # if false, a fake controller will be tracked, else a fake HMD
export SLAM_LOG=debug
export SLAM_CONFIG=$bsltdeps/basalt/data/monado/euroc.toml # Point to Basalt config file for Euroc
export OXR_DEBUG_GUI=1 # We will need the debug ui to start streaming the dataset
```

Finally, run the XR app and press start in the euroc player debug ui and you
should see a controller being tracked with Basalt from the euroc dataset.

## Using a RealSense Camera

After making sure that everything works by running the EuRoC datasets, it should
be possible to use the `realsense` driver from Monado to get any RealSense
camera that has an IMU and one or more cameras to get tracked with SLAM.
However, this was only tested on a D455, so if you are having problems with
another device, please open an issue. Also, open an issue if you manage to make
it work with other devices so that I can add it to this README.

### Overview of the Setup (D455)

Let's first assume you have a RealSense D455, which is the one that works with
the defaults. Even if you have another RealSense device follow this section, you
might at least get something working, although not at its best.

#### SLAM-Tracked RealSense Driver

Set these environment variables:

- `export RS_HDEV_LOG=debug`: Make our realsense device logs more verbose
- `export RS_TRACKING=2`: Only try to use "host-slam". See other options
  [here](https://gitlab.freedesktop.org/mateosss/monado/-/blob/64e70e76ad6d47e4bd1a0dfa164bff8597a50ce8/src/xrt/drivers/realsense/rs_prober.c#L33-39).
- `export SLAM_CONFIG=$bsltdeps/basalt/data/monado/d455.toml`:
  Configuration file for Basalt and the D455.

#### RealSense-Tracked Qwerty Driver

You now have a RealSense device that you can use to track another device, for
example, let's track a Qwerty HMD.

Set these environment variables to enable the qwerty driver:

```bash
export QWERTY_ENABLE=true QWERTY_COMBINE=true
```

And then modify your tracking overrides in your monado configuration file
(`~/.config/monado/config_v0.json`) by updating the json object with:

```js
{
  "tracking": {
    "tracking_overrides": [
      {
        "target_device_serial": "Qwerty HMD", // Or "Qwerty Left Controller"
        "tracker_device_serial": "Intel RealSense Host-SLAM",
        "type": "direct",
        "offset": {
          "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 },
          "position": { "x": 0, "y": 0, "z": 0 }
        },
        "xrt_input_name": "XRT_INPUT_GENERIC_TRACKER_POSE"
      }
    ],
  }
}
```

And that's it! You can now start an OpenXR application with Monado and get your
view tracked with your D455 camera.

### Non-D455 RealSense Devices

While I was unable to test other devices because I don't have access to them, it
should be possible to make them work by:

#### Configuring the RealSense Pipeline

[These
fields](https://gitlab.freedesktop.org/mateosss/monado/-/blob/9e1b7e2203ef49abb939cc8fc92afa16fcc9cb3a/src/xrt/drivers/realsense/rs_hdev.c#L118-129)
determine your RealSense streaming configuration, and
[these](https://gitlab.freedesktop.org/mateosss/monado/-/blob/9e1b7e2203ef49abb939cc8fc92afa16fcc9cb3a/src/xrt/drivers/realsense/rs_hdev.c#L40-50)
are their current defaults that work on a D455. You can change those fields by
setting any of them in your `config_v0.json` inside a `config_realsense_hdev`
field. Also note that as we already set `RS_HDEV_LOG=debug`, you should see the
values they are currently taking at the start of Monado.

For example, let's say you have a realsense device which has two fisheye cameras
that support streaming 640x360 at 30fps (a T265 I think), then a configuration
like this should work:

```js
"realsense_config_hdev": {
  "stereo": true,
  "video_format": 9, // 9 gets casted to RS2_FORMAT_Y8 (see https://git.io/Jzkfw), grayscale
  "video_width": 640, // I am assuming the T265 supports 640x360 streams at 30fps
  "video_height": 360,
  "video_fps": 30,
  "gyro_fps": 0, // 0 indicates any
  "accel_fps": 0,
  "stream_type": 4, // 4 gets casted to RS2_STREAM_FISHEYE (see https://git.io/Jzkvq)
  "stream1_index": -1, // If there were more than one possible stream with these properties select them, -1 is for auto
  "stream2_index": -1,
}
```

The particular values you could set here are very dependant on your camera. I
recommend seeing the values that get output by running the [rs-sensor-control
example](https://dev.intelrealsense.com/docs/rs-sensor-control) from the
RealSense API.

#### Configuring Basalt

As you might've noticed, we set `SLAM_CONFIG` to
`$bsltdeps/basalt/data/monado/d455.toml` which is [this](data/monado/d455.toml)
config file that I added for the D455. This file points to a [calibration
file](data/d455_calib.json) and a [VIO configuration
file](data/euroc_config.json).

For the tracking to be as good as possible you should set the
intrinsics/extrinsics of the device in a similar calibration file and point to
it with the `SLAM_CONFIG` config file. You can obtain that information from the
previously mentioned
[rs-sensor-control](https://dev.intelrealsense.com/docs/rs-sensor-control)
utility. Issues like [this
(T265)](https://gitlab.com/VladyslavUsenko/basalt/-/issues/52) and [this
(D435)](https://gitlab.com/VladyslavUsenko/basalt/-/issues/50) provide
configuration files tried by other users. Additionally Basalt provides custom
[calibration
tools](https://gitlab.com/VladyslavUsenko/basalt/-/blob/master/doc/Calibration.md)
that can work for any camera-IMU setup.

## Notes on Basalt Usage

- This fork is a first approximation to integrating Basalt with Monado, no fine
  tuning was made. The tracking is not perfect,
  [this](https://youtu.be/mIgRHmxbaC8) and [this](https://youtu.be/gxu3Ve8VCnI)
  show how it looks, as well as the problems that it has (difficulties with
  rotation-only movements, wiggliness on fast movements, etc)
- This fork only works with Stereo-IMU setups, but adapting Basalt to work with
  other configurations should feasible (see
  [granite](https://github.com/DLR-RM/granite)).
- Basalt is _fast_, the default D455 config uses 640x360 at 30fps but in my
  setup I could easily crank it up to 848x480 at 60fps and it worked flawlessly
  (see [`d455-848x480.toml`](data/monado/d455-848x480.toml)).
- While working on this fork a [new paper](https://arxiv.org/abs/2109.02182) and
  update to Basalt implemented a new marginalization method that should improve
  running speeds while maintaining accuracy. That should be easy to merge
  eventually.
