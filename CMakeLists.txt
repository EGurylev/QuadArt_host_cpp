cmake_minimum_required(VERSION 3.10)

set(PROJECT_VESRION 2.0.0)
project(quad_art VERSION ${PROJECT_VESRION})

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

add_subdirectory(src)

enable_testing()