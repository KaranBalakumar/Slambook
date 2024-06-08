include_directories("usr/local/eigen3")

find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_MODULE_PATH "/home/karan/slambook/Eigen3AndPangolin/examples/")
find_package(epoxy REQUIRED)
find_package(fmt)
# include_directories(${Pangolin_INCLUDE_DIRS} 
#                     ${OpenGL_INCLUDE_DIRS} 
#                     ${epoxy_INCLUDE_DIRS}
#                     ${Sophus_INCLUDE_DIRS} 
#                     )

add_executable(joinMap joinMap.cpp)
target_link_libraries(joinMap ${OpenCV_LIBS} ${Pangolin_LIBRARIES} fmt::fmt) 