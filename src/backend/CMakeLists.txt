project(slam-backend)

option(PROFILING "profiling" OFF)
option(JACOBIAN_ACCELERATOR "jacobian accelerator" OFF)
option(MULTIPARTICLE_ACCELERATOR "multiparticle jacobian accelerator" OFF)

include_directories(../../libs/eigen3)

set(BACKEND_FILES
        slam-backend.cpp
        Particle.cpp
        core.cpp
        utils.cpp
        SLAMBackendApplication.cpp
        algorithms/ekfslam.cpp
        algorithms/fastslam1.cpp
        algorithms/fastslam2.cpp
        plotting/NetworkPlot.cpp
        wrappers/slamwrapper.cpp
        wrappers/ekfslamwrapper.cpp
        wrappers/fastslam1wrapper.cpp
        wrappers/fastslam2wrapper.cpp
        wrappers/ParticleSLAMWrapper.cpp)

if(JACOBIAN_ACCELERATOR OR MULTIPARTICLE_ACCELERATOR)
    set(BACKEND_FILES
            ${BACKEND_FILES}
            AcceleratorHandler.cpp
            ${HLS_PROJECT}/impl/ip/drivers/toplevel_v1_0/src/xtoplevel_linux.c
            ${HLS_PROJECT}/impl/ip/drivers/toplevel_v1_0/src/xtoplevel.c
            )
    include_directories(${HLS_PROJECT}/impl/ip/drivers/toplevel_v1_0/src/)
endif()

add_executable(slam-backend ${BACKEND_FILES})

target_link_libraries(slam-backend zmqpp)

if(PROFILING)
    message("-- Profiling on")
    target_link_libraries(slam-backend -pg)
    target_compile_options(slam-backend PRIVATE -pg)
endif()

if(JACOBIAN_ACCELERATOR)
    add_definitions(-DJACOBIAN_ACCELERATOR=1)
endif()

if(MULTIPARTICLE_ACCELERATOR)
    add_definitions(-DMULTIPARTICLE_ACCELERATOR=1)
endif()


