project(slam-backend)

option(PROFILING "profiling" OFF)

file(GLOB_RECURSE BACKEND_FILES
        "*.cpp"
        )

include_directories(../../libs/eigen3)

add_executable(slam-backend ${BACKEND_FILES})

target_link_libraries(slam-backend zmqpp)

if(PROFILING)
    message("-- Profiling on")
    target_link_libraries(slam-backend -pg)
    target_compile_options(slam-backend PRIVATE -pg)
endif()

