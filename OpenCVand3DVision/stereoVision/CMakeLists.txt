include_directories("usr/local/eigen3")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenGL REQUIRED)
set(CMAKE_MODULE_PATH "/home/karan/slambook/Eigen3AndPangolin/examples/")
find_package(epoxy REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS} 
                    ${OpenGL_INCLUDE_DIRS} 
                    ${epoxy_INCLUDE_DIRS} 
                    )

add_executable(stereoVision stereoVision.cpp)
target_link_libraries(stereoVision ${OpenCV_LIBS} ${Pangolin_LIBRARIES})                    
