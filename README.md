# SLAM GUI

This program implements EKF SLAM, FastSLAM and FastSLAM 2 and presents them in a Qt GUI.

It is based on code from [here](https://github.com/bushuhui/fastslam) and [here](https://github.com/yglee/FastSLAM).

So far I have added a CMake build system and I am decoupling the algorithm from the GUI in order to do some profiling.

## Requirements
* Ubuntu 16.04
* CMake: `sudo apt install cmake`
* Qt4: `sudo apt install libqtcore4 libqtgui4 libqt4-dev`
* zmqpp: `sudo apt install libzmqpp3 libzmqpp-dev`

## Build
1. `mkdir build && cd build`
2. `cmake ..`
3. `make`

## Usage
```
./fastslam
    -m                  [s] input map file name
    -mode               [s] runing mode
        waypoints   : following given waypoints
        interactive : use keyboard to control movement
    -method             [s] SLAM method
        EKF1        : EKF SLAM 1
        FAST1       : FastSLAM 1
        FAST2       : FastSLAM 2
    -h  (print usage)
```


Examples:

`./fastslam -method FAST1 -mode interactive` (FastSLAM 1, user interactive)

` ./fastslam -method FAST2 -mode waypoints -m ../data/example_webmap.mat` (FastSLAM 2, following waypoints, map is "example_webmap.mat")

`./fastslam -method EKF1 -mode waypoints -m ../data/example_loop1.mat` (EKF SLAM, following waypoints, map is "example_loop1.mat")

`./fastslam -fn_screenshot ./screenshot/img -ww 400 -wh 300 -SWITCH_HEADING_KNOWN 0` (Generate video)

## Profiling 
1. `cmake -DPROFILING:BOOL=ON ..`
2. `make`
3. Run `./slam-backend` with appropriate options, such as `-m ../data/example_webmap.mat -method FAST2`
4. `gprof slam-backend gmon.out > analysis.txt`

## Issues
* Crash occurs when zooming or moving plot.
* Some segmentation faults every once in a while.
