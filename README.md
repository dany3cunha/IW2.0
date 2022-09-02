# IW2.0 - Virtual LIDAR based on the PointCloud parallel to the floor

This package create a virtual LIDAR using the pointcloud of a ZED2 camera.

### Prerequisites

- Ubuntu 18.04
- [ZED SDK **≥ 3.7**](https://www.stereolabs.com/developers/) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html)
- [PCL](https://pointclouds.org/downloads/)

#### Instalation

Open a terminal, clone the repository, update the dependencies and build the packages:

    $ cd ~/catkin_ws/src
    $ git clone --recursive https://github.com/dany3cunha/IW2.0
    $ cd ../
    $ rosdep install --from-paths src --ignore-src -r -y
    $ catkin_make -DCMAKE_BUILD_TYPE=Release
    $ source ./devel/setup.bash

### Run the Application

To launch the application:

    $ roslaunch zed-ros-app zed-ros-app.launch
