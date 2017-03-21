project(slam-gui)

find_package(PkgConfig)
find_package(Qt4)

set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(GUI_FILES
        plotting/WindowPlot.cpp
        slam-gui.cpp
        plotting/Controller.cpp
        ../../libs/qcustomplot/qcustomplot.cpp
        plotting/DataGatherer.cpp)

set(GUI_PKG_DEPS
        QtCore
        QtGui)

pkg_check_modules(GUI_DEPS REQUIRED ${GUI_PKG_DEPS})

include_directories(../../libs/eigen3)
include_directories(${GUI_DEPS_INCLUDE_DIRS})
add_definitions(${GUI_DEPS_CFLAGS})
link_libraries(${GUI_DEPS_LIBRARIES})
link_directories(${GUI_DEPS_LIBRARY_DIRS})

add_executable(slam-gui ${GUI_FILES})

target_link_libraries(slam-gui ${GUI_DEPS_LIBRARIES} zmqpp)
