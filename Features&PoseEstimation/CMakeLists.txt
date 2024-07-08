find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE "Release")
add_definitions("-DENABLE_SSE")
set(CMAKE_CXX_FLAGS "-std=c++17 -O2 ${SSE_FLAGS} -msse4")

add_executable(orb_cv orb_cv.cpp)
target_link_libraries(orb_cv ${OpenCV_LIBS})

add_executable(orb_self orb_self.cpp)
target_link_libraries(orb_self ${OpenCV_LIBS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

add_executable(triangulation triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS})

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt)

include_directories(${G2O_INCLUDE_DIRS})
include_directories("/usr/include/eigen3")

add_executable(pose_estimation_3d2d pose_estimation_3d2d)
target_link_libraries(pose_estimation_3d2d ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${OpenCV_LIBS} Sophus::Sophus fmt::fmt)

add_executable(pose_estimation_3d3d pose_estimation_3d3d)
target_link_libraries(pose_estimation_3d3d ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${OpenCV_LIBS} Sophus::Sophus fmt::fmt)