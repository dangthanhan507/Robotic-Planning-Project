# Top-Level cmake
cmake_minimum_required(VERSION 3.10.2)
project(workspace)

#load libraries
find_package(drake CONFIG REQUIRED)
find_package(gflags REQUIRED)

#run kuka submodule
link_directories(kuka/include)
add_executable(kuka_main kuka/src/main.cpp)
target_link_libraries(kuka_main drake::drake gflags)

#.... and so on and so forth.