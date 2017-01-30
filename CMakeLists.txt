cmake_minimum_required(VERSION 3.5)
project(slam-gui)

find_package(PkgConfig)
find_package(Qt4)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse4 -g -rdynamic -D__STDC_CONSTANT_MACROS -Wno-deprecated-declarations")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lz -lpthread")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(SOURCE_FILES
        src/wrappers/ekfslamwrapper.cpp
        src/wrappers/fastslam1wrapper.cpp
        src/wrappers/fastslam2wrapper.cpp
        src/algorithms/ekfslam.cpp
        src/algorithms/fastslam1.cpp
        src/algorithms/fastslam2.cpp
        src/particle.cpp
        src/core.cpp
        src/plot.cpp
        src/wrappers/slamwrapper.cpp
        src/utils.cpp
        src/main.cpp
        src/algorithms/ekfslam.cpp
        libs/qcustomplot/qcustomplot.cpp
        )

set (PKG_DEPS
        QtCore
        QtGui
        )

pkg_check_modules (DEPS REQUIRED ${PKG_DEPS})

add_definitions(${DEPS_CFLAGS})
link_libraries(${DEPS_LIBRARIES})
link_directories(${DEPS_LIBRARY_DIRS})

link_directories(${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(libs/eigen3)

add_executable(slam-gui ${SOURCE_FILES})

target_link_libraries(slam-gui ${DEPS_LIBRARIES})

