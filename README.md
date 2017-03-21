# SLAM GUI

This program implements EKF SLAM, FastSLAM and FastSLAM 2 and presents them in a Qt GUI.

It is based on code from [here](https://github.com/bushuhui/fastslam) and [here](https://github.com/yglee/FastSLAM).

So far I have added a CMake build system and I am decoupling the backend from the GUI to allow the backend to run on an 
embedded system and connect over the network to a GUI running on a desktop.

The backend uses an FPGA hardware accelerator written in C++ using Vivado HLS and intended to run on the Xilinx Zynq 
platform (currently on a Digilent Zybo board). You can find the Vivado project files 
[here](https://github.com/matzipan/slam-xilinx). More information below.

## Requirements
* Ubuntu 16.04
* CMake: `sudo apt install cmake`
* Qt4: `sudo apt install libqtcore4 libqtgui4 libqt4-dev`
* zmq: `sudo apt install libzmq5 libzmq3-dev`

## Build
1. `mkdir build && cd build`
2. `cmake ..`. Enable debug build type using the following argument: `-DCMAKE_BUILD_TYPE=Debug`
3. `make`
4. The executables paths are `src/backend/slam-backend` and `src/gui/slam-gui`.

## Usage

You have to start `slam-gui` first. This will launch the GUI which you can connect to on port 4242. 

You will then have to launch `slam-backend`, with any of the following arguments:
```
./slam-backend
    -m                  [s] input map file name
    -mode               [s] runing mode
        waypoints   : following given waypoints
        interactive : use keyboard to control movement
    -method             [s] SLAM method
        EKF1        : EKF SLAM 1
        FASTSLAM1       : FastSLAM 1
        FASTSLAM2       : FastSLAM 2
    -h  (print usage)
```

Examples:

`./slam-backend -method FASTSLAM1 -mode interactive` (FastSLAM 1, user interactive)

`./slam-backend -method FASTSLAM2 -mode waypoints -m ../data/example_webmap.mat` (FastSLAM 2, following waypoints, map is "example_webmap.mat")

`./slam-backend -method EKF1 -mode waypoints -m ../data/example_loop1.mat` (EKF SLAM, following waypoints, map is "example_loop1.mat")

`./slam-backend -fn_screenshot ./screenshot/img -ww 400 -wh 300 -SWITCH_HEADING_KNOWN 0` (Generate video)
f
## Profiling 
1. `cmake -DPROFILING:BOOL=ON ..`
2. `make`
3. Run `./slam-backend` with appropriate options, such as `-m ../data/example_webmap.mat -method FAST2`
4. `gprof slam-backend gmon.out > analysis.txt`

## Cross-compilation and FPGA acceleration

To cross-compile the software to run on the Zynq ARM, several arguments need to be passed to `cmake`:
* Point `cmake` to the toolchain definition file: `-DCMAKE_TOOLCHAIN_FILE=../crosscompile-zynq.cmake`
* The toolchain needs to be pointed towards the root file system to use for libraries and header definition files: 
`-DROOTFS=/media/matzipan/Xilinx/project/rootfs`
* To enable the FPGA acceleration for Jacobian computation, pass the following switch: `-DJACOBIAN_ACCELERATOR=on`
* The Jacobian accelerator needs to be pointed to the Vivado HLS project, where it looks for the generated 
UIO driver files to use: `-DHLS_PROJECT=/home/matzipan/Workspace/project/zynq-slam/jacobian-accelerator/jacobian-accelerator/`
* To disable building the GUI for use in an embedded system, use: `-DBUILD_GUI=off`

The `computeJacobians` in `core.cpp` is currently the only one accelerated. The `AcceleratorHandler` class is used as a wrapper
for interaction with the FPGA device. This class `mmap`s 256 KBs of memory at physical address `0xFFFC0000`, which corresponds to 
the high range of On-Chip Memory (OCM) in the Zynq SoC. Thus, it assumes the SoC is configured to place the entirety OCM memory 
at the high phyiscal address. Through the Userspace I/O (UIO) kernel mechanism, the `AcceleratorHandler` uses the driver generated
by Vivado HLS to `mmap` the device control region and interact with it.

## Issues
* Crash sometimes occurs when zooming or moving plot.
* Load the configuration and perform simulation and control on the GUI side of things and only send range and bearing 
information to backend.