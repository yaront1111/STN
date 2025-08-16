cmake_minimum_required(VERSION 3.10)
project(stn_core)

set(CMAKE_CXX_STANDARD 17)

include_directories(include)

add_executable(stn_core
    core/main.cpp
)
