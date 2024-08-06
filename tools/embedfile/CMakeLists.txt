cmake_minimum_required(VERSION 3.4)

project(embedfile LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(embedfile embedfile.cpp)
