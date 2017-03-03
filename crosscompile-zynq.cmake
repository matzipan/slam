set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

set(GCC_PREFIX arm-xilinx-linux-gnueabi-)

find_program(CMAKE_C_COMPILER NAMES ${GCC_PREFIX}gcc)
find_program(CMAKE_CXX_COMPILER NAMES ${GCC_PREFIX}g++)
find_program(CMAKE_LINKER NAMES ${GCC_PREFIX}ld)
find_program(CMAKE_AR NAMES ${GCC_PREFIX}ar)

link_directories(${ROOTFS}/lib)
include_directories(${ROOTFS}/include)

set(TARGET "zynq")