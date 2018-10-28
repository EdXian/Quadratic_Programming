# Quadratic Programming
It is a simple test for QP(quadratic programming) problem.This package can help you generate 2d path for manipulator or quadcoptor. The package is implemented by C APi in python-cvxopt and Eigen. 


## Testbed
* Ubuntu 16.04
* intel i7-7700
* gcc 5.4.0
* cmake v3.5.1


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

cd ~catkin_ws

catkin_make

rosrun qptrajectory qptest

rqt_plot

```


