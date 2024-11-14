include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} ${OpenGL_INCLUDE_DIRS} ${epoxy_INCLUDE_DIRS})
add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError Sophus::Sophus ${Pangolin_LIBRARIES})