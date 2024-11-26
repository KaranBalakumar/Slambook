cmake_minimum_required(VERSION 2.8)
project(Slambook)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

add_subdirectory(BundleAdjustment)
add_subdirectory(Features_and_PoseEstimation)
add_subdirectory(OpticalFlow)
add_subdirectory(LoopClosure)
add_subdirectory(CPPandCmake)
add_subdirectory(NonlinearOptimization)
add_subdirectory(Sophus_LieGroups)
add_subdirectory(Eigen3AndPangolin)
add_subdirectory(OpenCVand3DVision)