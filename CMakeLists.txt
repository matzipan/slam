cmake_minimum_required(VERSION 3.3)
project(slam)

option(BUILD_GUI "build-gui" ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -rdynamic -D__STDC_CONSTANT_MACROS -Wno-deprecated-declarations")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lpthread")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86_64 OR ${CMAKE_SYSTEM_PROCESSOR} STREQUAL x86)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lz")
endif()

link_directories(${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})

add_subdirectory(libs/zmqpp)
add_subdirectory(src/backend)
if(BUILD_GUI)
    add_subdirectory(src/gui)
endif()
