# PID Control (IN PROGRESS)
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---

This Project is submitted as part of the Udacity Self-Driving Car Nanodegree.

For it the goal is to write C++ code implementing a PID controller for a vehicle driving around a virtual circuit.

The source code for this project is submitted as part of this Git repo (in the [src](/src) folder). A detailed explanation is provided in a separate writeup, that documents also the results obtained.  

Dependencies
---
First of all, this project involves the Udacity Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases).

Furthermore, this repository includes two files that can be used to set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets) for either Linux or Mac systems. For windows you can use either Docker, VMware, or even [Windows 10 Bash on Ubuntu](https://www.howtogeek.com/249966/how-to-install-and-use-the-linux-bash-shell-on-windows-10/) to install uWebSocketIO.

Other important dependencies are:

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

Compiling the Code
---

The code is intended to be compiled using CMake and Make. The guideline followed here is to have a `Release` and `Debug` build profile, as suggested in [here](https://stackoverflow.com/questions/7724569/debug-vs-release-in-cmake) and in the references within.

After having cloned this repo and taken care of the dependencies outlined here above, for the `Debug` case you should need to run the following commands from the project's root: 

```
  mkdir debug
  cd debug
  cmake -DCMAKE_BUILD_TYPE=Debug ..
  make
```

while for the `Release` case, again from the project's root you will need to run: 

```
  mkdir release
  cd release
  cmake -DCMAKE_BUILD_TYPE=Release ..
  make
```

CMake will take care of the dependencies. In the `Debug` case a flag has been deined in the [CMakeFiles.txt](src/CmakeLists.txt) (line) to allow printout on screen of various messages:

```
   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DPID_DEBUG")
```

An example of this can be found in [main.cpp](src/main.cpp) on lines:

