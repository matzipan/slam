project(zmqpp)

find_package(PkgConfig)

file(GLOB ZMQPP_FILES
        "*.cpp"
        )

add_library(zmqpp STATIC ${ZMQPP_FILES})

target_include_directories(zmqpp PUBLIC .)

if(${TARGET} MATCHES "zynq")
    find_library(ZMQ_LIBRARY zmq HINTS ${ROOTFS}/lib)
    target_link_libraries(zmqpp PUBLIC ${ZMQ_LIBRARY})
    target_compile_definitions(zmqpp PRIVATE ZMQ_BUILD_DRAFT_API=1)
else()
    set (ZMQPP_PKG_DEPS
            libzmq)

    pkg_check_modules (ZMQPP_DEPS REQUIRED ${ZMQPP_PKG_DEPS})

    target_link_libraries(zmqpp PUBLIC ${ZMQPP_DEPS_LIBRARIES})

    include_directories(${ZMQPP_DEPS_INCLUDE_DIRS})
    add_definitions(${ZMQPP_DEPS_CFLAGS})
    link_directories(${ZMQPP_DEPS_LIBRARY_DIRS})
endif()

