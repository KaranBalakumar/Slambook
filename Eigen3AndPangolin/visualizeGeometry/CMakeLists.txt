include_directories("/usr/include/eigen3")
set(CMAKE_CXX_STANDARD 14)

find_package(Pangolin REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(visualizeGeometry visualizeGeometry.cpp)
target_link_libraries(visualizeGeometry ${Pangolin_LIBRARIES})

