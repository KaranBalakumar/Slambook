include_directories("/usr/include/eigen3")
add_executable(coordinateTransform coordinateTransform.cpp)

set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_MODULE_PATH "/home/karan/slambook/Eigen3AndPangolin/examples/")
find_package(epoxy REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS} ${epoxy_INCLUDE_DIRS})
add_executable(plotTrajectory plotTrajectory.cpp)
target_link_libraries(plotTrajectory ${Pangolin_LIBRARIES})

