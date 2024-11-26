include_directories("/usr/include/eigen3")

find_package(Sophus REQUIRED)

add_executable(useSophus useSophus.cpp)
target_link_libraries(useSophus Sophus::Sophus)

add_subdirectory(examples)