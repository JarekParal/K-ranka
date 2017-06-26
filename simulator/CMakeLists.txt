cmake_minimum_required(VERSION 2.8)

project(simulator)

file(GLOB_RECURSE src "*.cpp" "*.hpp")
include_directories("../firmware")
add_definitions(-DHACKME_SIMULATOR -std=c++1z)
add_executable(simulator ${src} "../firmware/bfgrid.cpp")