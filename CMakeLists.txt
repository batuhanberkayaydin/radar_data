cmake_minimum_required(VERSION 3.5)

project(radar_data)

# Find OpenCV package
find_package(OpenCV REQUIRED)

# Add executable
add_executable(radar_data decode_ethernet.c main.cpp)

# Link OpenCV libraries
target_link_libraries(radar_data ${OpenCV_LIBS})