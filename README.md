
# LiDAR SLAM

- [Introduction and contents](#introduction-and-contents)
- [Core SLAM lib](#core-slam-lib)
  - [Dependencies](#dependencies)
  - [Installation](#installation)
- [ROS wrapping](#ros-wrapping)
  - [Dependencies](#dependencies-1)
  - [Installation](#installation-1)
  - [Usage](#usage)
- [ParaView wrapping](#paraview-wrapping)
  - [Dependencies](#dependencies-2)
  - [Installation](#installation-2)
  - [Usage](#usage-1)
  - [Use SLAM in LidarView](#use-slam-in-lidarview)

## Introduction and contents

This repo contains LiDAR-only visual SLAM developped by Kitware, as well as ROS and ParaView wrappings for easier use.

It has been successfully tested on data from several common LiDAR sensors:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- RoboSense (RS-LiDAR-16)
- Hesai (Pandar128)

Have a look at our [SLAM demo video](https://vimeo.com/524848891)!

This codebase is under active development. If you're interested by new features, new sensors' support or any project that could be using this SLAM, do not hesitate to contact us at kitware@kitware.com.

Repo contents :
- `slam_lib/` : core *LidarSlam* library containing SLAM algorithm and other utilities.
- `ros_wrapping/` : ROS packages to enable SLAM use on a ROS system.
- `paraview_wrapping/` : ParaView plugin to enable SLAM use with ParaView/LidarView.
- `ci/` : continuous integration files to automatically build and check *LidarSlam* lib.
- `CMakeLists.txt` : *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.

## Core SLAM lib

### Dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency | Minimum tested Version |
| :--------: | :--------------------: |
| Eigen3     | 3.3.4                  |
| Ceres      | 1.13.0                 |
| PCL        | 1.8                    |
| nanoflann  | 1.3.0                  |
| g2o*       | 1.0.0 (master)         |
| OpenMP*    | 2.0                    |

## Eigen3
  - ### Dependencies
```bash
sudo apt-get install libopenblas-dev
sudo apt-get install --no-install-recommends libboost1.58-all-dev
sudo apt-get install libx11-dev
sudo apt-get install libgl1-mesa-dev 
sudo apt-get install libglu1-mesa-dev 
sudo apt-get install freeglut3-dev
sudo apt-get install doxygen
sudo apt-get install cmake
sudo apt-get install libeigen3-dev
sudo wget https://nchc.dl.sourceforge.net/project/glew/glew/2.1.0/glew-2.1.0.tgz --no-check-certificate
sudo tar -xzvf glew-2.1.0.tgz
cd glew-2.1.0/
sudo make 
sudo make install
sudo ldconfig -v
```
  - ### Install Eigen 3.3.5
```bash
sudo wget https://github.com/eigenteam/eigen-git-mirror/archive/3.3.5.tar.gz
sudo tar -xzvf 3.3.5.tar.gz 
sudo mv eigen-git-mirror-3.3.5/ eigen-3.3.5/
cd eigen-3.3.5/
sudo mkdir build
sudo cmake ..
sudo make
```

## Ceres Solver 
  - ### Dependencies
```bash
# CMake
sudo apt-get install cmake
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev
# BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse and CXSparse (optional)
# - If you want to build Ceres as a *static* library (the default)
#   you can use the SuiteSparse package in the main Ubuntu package
#   repository:
sudo apt-get install libsuitesparse-dev
# - However, if you want to build Ceres as a *shared* library, you must
#   add the following PPA:
sudo add-apt-repository ppa:bzindovic/suitesparse-bugfix-1319687
sudo apt-get update
sudo apt-get install libsuitesparse-dev
```
  - ### Install ceres-solver
```bash 
git clone https://ceres-solver.googlesource.com/ceres-solver
mkdir ceres-bin
cd ceres-bin
cmake ../ceres-solver
make -j4
make test
# Optionally install Ceres, it can also be exported using CMake which
# allows Ceres to be used without requiring installation, see the documentation
# for the EXPORT_BUILD_DIR option for more information.
sudo make install
```
## PCL
  - ### Dependencies
```bash
#Boost
sudo apt-get install libboost-all-dev
#Eigen DONE
#FLANN DONE
#VTK   DONE
```
  - ### Install PCL
```bash
git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl
mkdir build && cd build
cmake ..
make -j4
sudo make -j4 install
```

## Nanoflann
  - ### Dependencies
```bash
sudo apt-get install build-essential cmake libgtest-dev libeigen3-dev
```
  - ### Install Nanoflann
```bash
git clone https://github.com/jlblancoc/nanoflann.git
cd nanoflann
mkdir build && cd build 
cmake .. 
make
sudo make install
```
## VTK
  - ### Dependencies
```bash
sudo apt install cmake libavcodec-dev libavformat-dev libavutil-dev libboost-dev libdouble-conversion-dev libeigen3-dev libexpat1-dev libfontconfig-dev libfreetype6-dev libgdal-dev libglew-dev libhdf5-dev libjpeg-dev libjsoncpp-dev liblz4-dev liblzma-dev libnetcdf-dev libnetcdf-cxx-legacy-dev libogg-dev libpng-dev libpython3-dev libqt5opengl5-dev libqt5x11extras5-dev libsqlite3-dev libswscale-dev libtheora-dev libtiff-dev libxml2-dev libxt-dev qtbase5-dev qttools5-dev zlib1g-dev
```
  - ### Install VTK
```bash
git clone gitlab.kitware.com:vtk/vtk.git
cd vtk
git checkout v8.2.0
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$HOME/vtk-inst \
    -DCMAKE_INSTALL_RPATH=$HOME/vtk-inst \
    -DVTK_Group_Qt=ON \
    -DVTK_QT_VERSION=5 \
    -DVTK_Group_Imaging=ON \
    -DVTK_Group_Views=ON \
    -DModule_vtkRenderingFreeTypeFontConfig=ON \
    -DVTK_WRAP_PYTHON=ON \
    -DVTK_PYTHON_VERSION=3 \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6 \
    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so \
    -DBUILD_TESTING=OFF \
    -DVTK_USE_SYSTEM_LIBRARIES=ON \
    -DVTK_USE_SYSTEM_LIBPROJ4=OFF \
    -DVTK_USE_SYSTEM_GL2PS=OFF \
    -DVTK_USE_SYSTEM_LIBHARU=OFF \
    -DVTK_USE_SYSTEM_PUGIXML=OFF \
    -DCMAKE_BUILD_TYPE=Release \
    ..
make -j$(($(nproc) - 1))
sudo make install
```
## g2o
  - ### Dependencies
```bash
sudo apt install libeigen3-dev
sudo apt install cmake
```
  - ### Install g2o
```bash
https://github.com/RainerKuemmerle/g2o.git
cd g2o
mkdir build && cd build
cmake ..
make 
sudo make install
```


(*) optional dependencies :

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.

### Installation

To build only *LidarSlam* lib, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make
make install
```

The *LidarSlam* lib has been tested on Linux, Windows and OS X.

## ROS wrapping

### Dependencies

Ensure all *LidarSlam* dependencies are respected. Specific ROS packages dependencies are listed in the table below along with the version used during development and testing.

| Dependency     | Tested Version | Install (`sudo apt-get install <pkg>`)                                           |
|:--------------:|:--------------:|:--------------------------------------------------------------------------------:|
| ROS            | melodic        | `ros-melodic-desktop-full` and [tutorial](http://wiki.ros.org/ROS/Installation)  |
| velodyne_pcl   | 1.6.1          | `ros-noetic-velodyne-pcl` or [git repo](https://github.com/ros-drivers/velodyne) |
| gps_common     | 0.3.0          | `ros-$ROS_DISTRO-gps-common`                                                     |
| geodesy        | 0.5.3          | `ros-$ROS_DISTRO-geodesy`                                                        |

Please note that the ROS Velodyne driver with minimum version 1.6 is needed.
Be careful, this ROS Velodyne driver 1.6 is not backward-compatible with previous versions.
If you're running on Ubuntu 20 / ROS Noetic, you can install the new Velodyne driver using the command `sudo apt install ros-noetic-velodyne ros-noetic-velodyne-pcl`.
If running on previous versions of Ubuntu/ROS (18/Melodic and below), you need to compile this driver from source : just clone the [git repo](https://github.com/ros-drivers/velodyne) in your catkin worskpace sources, it will be automatically built with next  `catkin_make`.

### Installation

Clone this git repo directly into your catkin directory, and run `catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo` or `catkin_make -DCMAKE_BUILD_TYPE=Release` (to turn on optimizations, highly recommended when using Eigen). It will automatically build *LidarSlam* lib before ROS packages.

The ROS wrapping has been tested on Linux only.

### Usage

```bash
roslaunch lidar_slam slam.launch
roslaunch lidar_slam slam.launch gps:=true   # if GPS/SLAM calibration has to be run
```

See [ros_wrapping/lidar_slam/README.md](ros_wrapping/lidar_slam/README.md) for more details.

## ParaView wrapping

### Dependencies

Ensure all *LidarSlam* dependencies are respected. Specific dependencies are listed in the table below along with the version used during development and testing.

| Dependency | Tested Version |
| :--------: | :------------: |
| ParaView   | 5.4 and 5.6    |

Be careful to use and link to the same libraries as ParaView/LidarView's (especially with VTK, Eigen, PCL, Ceres, nanoflann, etc.). Otherwise, if different flags or modules were enabled, some troubles may arise at build time, or it could lead to version mismatch and segfault at runtime.

For example, if PCL is built with `pcl_visualization` module, it must link to the same VTK than the one used by ParaView.

### Installation

To build *LidarSlam* lib and this ParaView plugin *LidarSlamPlugin*, just `cd` to this repo root dir and run :

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DSLAM_PARAVIEW_PLUGIN:BOOL=ON
make
make install
```

The *LidarSlamPlugin* Paraview wrapping has been tested on Linux, Windows and OS X.

### Usage

- Open ParaView
- **Tools** tab > **Manage Plugins** > **Load New**
- Browse to the `<install>/lib/` dir and select `libLidarSlamPlugin.so` or `LidarSlamPlugin.dll`
- Load LiDAR frames and LiDAR calibration to use
- Select the frames in the Pipeline Browser, instantiate a SLAM filter, and apply it.

Currently, all features are not available in ParaView plugin. Features such as GPS/LiDAR calibration, pose graph optimization or temporal logging are only supported in ROS wrapping. However, ParaView plugin is useful to play with SLAM, interactively try out parameters, visualize and export results.

### Use SLAM in LidarView

This *LidarSlamPlugin* is natively included in [LidarView](https://www.paraview.org/lidarview/). For more detailed information on how to enable and use SLAM filter in LidarView, see [paraview_wrapping/doc/How_to_SLAM_with_LidarView.md](paraview_wrapping/doc/How_to_SLAM_with_LidarView.md).

Pre-built binaries of LidarView with this SLAM plugin are available for download [here](https://drive.google.com/drive/folders/1ouNd3KD2p62a0XqRu4eJ2Tus6LJ-LBE8?usp=sharing).
