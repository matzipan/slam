cmake_minimum_required(VERSION 3.5)
project(slam-gui)

find_package(PkgConfig)
find_package(Qt4)

option(PROFILING "profiling" OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -msse4 -g -rdynamic -D__STDC_CONSTANT_MACROS -Wno-deprecated-declarations")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lz -lpthread")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(BACKEND_FILES
        src/backend/slam-backend.cpp
        src/backend/Particle.cpp
        src/backend/core.cpp
        src/backend/utils.cpp
        src/backend/wrappers/slamwrapper.cpp
        src/backend/wrappers/ekfslamwrapper.cpp
        src/backend/wrappers/fastslam1wrapper.cpp
        src/backend/wrappers/fastslam2wrapper.cpp
        src/backend/wrappers/ParticleSLAMWrapper.cpp
        src/backend/algorithms/ekfslam.cpp
        src/backend/algorithms/fastslam1.cpp
        src/backend/algorithms/fastslam2.cpp
        src/backend/plotting/NetworkPlot.cpp
        src/backend/SLAMBackendApplication.cpp
        libs/qcustomplot/qcustomplot.cpp)

set(GUI_FILES
        src/gui/plotting/WindowPlot.cpp
        src/gui/slam-gui.cpp
        src/gui/plotting/PlotController.cpp
        libs/qcustomplot/qcustomplot.cpp)

set (PKG_DEPS
        QtCore
        QtGui)

pkg_check_modules (DEPS REQUIRED ${PKG_DEPS})

add_definitions(${DEPS_CFLAGS})
link_libraries(${DEPS_LIBRARIES})
link_directories(${DEPS_LIBRARY_DIRS})

link_directories(${CMAKE_BINARY_DIR})

include_directories(${CMAKE_BINARY_DIR})
include_directories(${CMAKE_SOURCE_DIR})
include_directories(libs/eigen3)

add_executable(slam-backend ${BACKEND_FILES})
add_executable(slam-gui ${GUI_FILES})

target_link_libraries(slam-backend ${DEPS_LIBRARIES} zmqpp)
target_link_libraries(slam-gui ${DEPS_LIBRARIES} zmqpp)

if(PROFILING)
    target_link_libraries(slam-backend -pg)
    target_compile_options(slam-backend PRIVATE -pg)
endif()

