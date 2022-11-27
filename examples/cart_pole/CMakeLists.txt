cmake_minimum_required(VERSION 3.10.2)
project(pendulum)

find_package(drake CONFIG REQUIRED)
#ExternalProject_Add(drake)
find_package(gflags REQUIRED)

add_executable(lqr_main src/lqr_main.cpp)
target_link_libraries(lqr_main drake::drake gflags)