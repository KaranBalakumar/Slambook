# Visual Odometry and VisualSLAM basics in C++
This repository contains basics and implementations required for Visual Odometry and VisualSLAM. This repository contains,
- Very Basic C++ and CMake examples.
- Implementation of ```Eigen3``` template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
- Simple **3D Trajectory plotter** using ```Eigen3``` and ```Pangolin```.
- Display camera's Rotation matrix, Euler Angles, Translation vector and Quaternion visualizer.
- Intro to ```OpenCV``` and mapping point clouds from stereo and RGBD camera images.
- Examples of employing ```Ceres```, ```g2o``` libraries for non linear optimization through curve fitting.
- ```ORB feature``` extraction using ```OpenCV``` and another **ORB feautre extraction from scratch** for a particular **ORB pattern**
- ```Pose estimation of Camera``` using **Essential and Fundamental Matrix** and then decompose **E** to find **R** and **t**, those whole program is solved using algorithms provided by ```OpenCV```.