cmake_minimum_required(VERSION 3.16)
project(final_project)
# Set the project root directory
set(PROJECT_ROOT_DIR ${CMAKE_SOURCE_DIR})
# Pass the project root directory to the C++ code
add_compile_definitions(PROJECT_ROOT_DIR="${PROJECT_ROOT_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
find_package(Open3D REQUIRED HINTS ${PROJECT_SOURCE_DIR}/open3d/lib/cmake)
add_subdirectory(dataloader)
add_subdirectory(viewer)
add_subdirectory(app)