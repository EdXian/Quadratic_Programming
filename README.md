# Quadratic Programming
It is a simple test for QP(quadratic programming) problem.This package can help you generate 2d path for manipulator or quadcoptor. The package is implemented by C APi in python-cvxopt and Eigen. 


## Testbed
#### 1
* Ubuntu 16.04
* intel i7-7700
* gcc 5.4.0
* cmake v3.5.1

#### 2
* Ubuntu 16.04
* intel i9-7980XE
* gcc 5.4.0
* cmake v3.5.2

## Requirements

It is necessary to install python-cvxopt lib and eigen lib. 

```
sudo apt-get install libeigen3-dev
sudo apt-get install python-cvxopt

```

## How to use?
The project can be directly imported by Qt5.9. 
1. Cmake
```
cd qpc 
mkdir build && cd build
cmake ..
make
./cvxqp
```
2. Qt 5.9
```
cd QPsolve
mkdir build && cd build
qmake ..
make
./QPSolve
```
3. rospackage
```
clone folder "qptrajectory_ros" to ~/catkin_ws/src

cd ~/catkin_ws

catkin_make

rosrun qptrajectory qptest

rqt_plot

```

## Troubleshoot

### `catkin build` ( `catkin_make` ) compile error

```
Errors     << qptrajectory:make ~/catkin_ws/logs/qptrajectory/build.make.005.log             
//usr/lib/x86_64-linux-gnu/libapr-1.so.0: undefined reference to `uuid_generate@UUID_1.0'
collect2: error: ld returned 1 exit status
make[2]: *** [~/catkin_ws/devel/.private/qptrajectory/lib/qptrajectory/qptest] Error 1
make[1]: *** [CMakeFiles/qptest.dir/all] Error 2
make: *** [all] Error 2
```
##### It might suggest that the `libuuid` version is too old, try to fix it by `conda install -c conda-forge libuuid `
##### [reference](https://github.com/uzh-rpg/rpg_esim/issues/7)



