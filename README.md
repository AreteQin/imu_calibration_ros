# IMU-TK-ROS: Inertial Measurement Unit Calibration ToolKit #

The C++ IMU-TK-ROS Library is forked from [
imu_tk](https://github.com/Kyle-ak/imu_tk)
to calibrate MEMS-based inertial navigation units using 
ROS bag files. QT4 is not required anymore. You can generate 
two types of simulation data: 1) IMU static data and 2) IMU data 
intervals.

## References ##

Papers Describing the Approach:

D. Tedaldi, A. Pretto and E. Menegatti, "A Robust and 
Easy to Implement Method for IMU Calibration without 
External Equipments". In: Proceedings of the IEEE 
International Conference on Robotics and Automation 
(ICRA 2014), May 31 - June 7, 2014 Hong Kong, China, 
Page(s): 3042 - 3049 ([PDF](./A_robust_and_easy_to_implement_method_for_IMU_calibration_without_external_equipments.pdf))

## License ##

IMU-TK is licensed under the BSD License.
IMU-TK is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the licenses for more details.

## Requirements ##

The code is tested on Ubuntu 20.04 and 
ROS Noetic. IMU-TK requires the 
following tools and libraries: CMake, 
Eigen3, Ceres Solver, OpenGL, and Glog. 
To install these required packages on 
Ubuntu, use the terminal command:

```
#!bash

sudo apt-get install build-essential 
cmake libeigen3-dev freeglut3-dev gnuplot
```
and follow this [guide](http://ceres-solver.org/building.html) to install Ceres Solver.

## Building ##

To build IMU-TK-ROS on Ubuntu, type in a terminal the following command sequence.

```
#!bash

cd catkin_ws
catkin_make
```

## Test ##
```
#!bash

./test_imu_calib ros_bag_file.bag

```

## Contact information ##

Qiaomeng Qin [qinqiaomeng@outlook.com](qinqiaomeng@outlook.com)