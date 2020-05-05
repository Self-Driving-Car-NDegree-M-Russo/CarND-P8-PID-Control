# PID Control
[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

---
## Overview

This Project is submitted as part of the Udacity Self-Driving Car Nanodegree.

For it the goal is to write C++ code implementing a PID controller for a vehicle driving around a virtual circuit.

---
## Difference between Branches

At the present moment this repo has two different branches: `master` and `log_and_test`. The main difference between the two branch is the implementation, in the second one, of a test suite and a logger functionality, both realised using the [Boost libraries](https://www.boost.org/).
The `master` branch does not present this dependency, even if, as it will be explained in the following section, still has been designed to provide a debug capability.

The reason for maintaining the branches separated is to keep the code for the actual project evaluation as simple as possible (the project does not actually require the implementation of logging/tests) and limit the number of dependencies (among the other thing, the code in `log_and_test` hasn't really beentested on multiple systems, and so at the present stage there is no guarantee on its portability).
Nonetheless, I am a firm believer in TDD (Test Driven Development) and I wanted to experiment on making these functionalities available for some C++ source code.

The present README is common to the two branches, even if the rest of the repo changes: in the following I will describe how to build the code in the two cases. The source code for this project is in the [src](/src) folder: a detailed explanation is provided in a separate writeup, that documents also the results obtained.  

---
## `master` Branch

The `master` branch is available [here](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/tree/master) - the writeup describing the project is [here](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/master/PID_writeup.md).

### _Dependencies_

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

### _Compiling the Code_

The code is intended to be compiled using CMake and Make. The guideline followed here is to have a `Release` and `Debug` build profile, as suggested in [here](https://stackoverflow.com/questions/7724569/debug-vs-release-in-cmake) and in the references within.

After having cloned this repo and taken care of the dependencies outlined above, for the `Debug` case you should need to run the following commands from the project's root: 

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

CMake will take care of the dependencies. In the `Debug` case a flag (`PID_DEBUG`) has been defined in the [CMakeLists.txt](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/master/CMakeLists.txt) (line 10) to allow printout on screen of various messages:

```
   set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DPID_DEBUG")
```

An example of this can be found in [main.cpp](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/master/src/main.cpp) on lines (89-92):

```
  // DEBUG
  #ifdef PID_DEBUG
     std::cout << "CTE: " << cte << " Steering Value: " << steer_value
          << std::endl;
  #endif
```          

### _Running  the code_

In order to run the code in [`main.cpp`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/master/src/main.cpp), starting from the `debug` or `release` folder just type:

```sh
  ./pid 
```

Depending in which folder you are you will/will not see debug messages on the console.

---
## `log_and_test` Branch

The `log_and_test` branch is available [here](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/tree/log_and_test) - the writeup describing the project is [here](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/PID_writeup.md).

### _Dependencies_

Beyond ALL the dependencies applicable to the `master` branch, the `log_and_test` one also requires the **Boost** libraries.

[Boost](https://www.boost.org/) provides portable C++ source libraries that cover a very wide span of applications, from Linear Algebra to Image Processing. The code in this repo is focused on making use of the Logging and Testing capabilities

#### _Download and Install Boost_
The latest version of Boost can be downloaded from [here](https://www.boost.org/users/download/). The code in this repo has been tested up to ver. 1.72. The instructions on how to get started can be found [here](boost.org/doc/libs/1_72_0/more/getting_started/unix-variants.html) for Unix variants (Linux/MacOS) and [here](https://www.boost.org/doc/libs/1_72_0/more/getting_started/windows.html) for Windows systems.

Summarizing the main instructions _for Unix systems_ we have:

1. Download `boost_X_XX_0.tar.bz2`;
2. Create a directory where to put the Boost installation;
3. From the directory where you want to put the Boost installation, execute:

```sh
  tar --bzip2 -xf /path/to/boost_X_XX_0.tar.bz2
```

4. In the same directory execute:

```sh
  ./bootstrap.sh --prefix=path/to/installation/folder
```

5. Change directory to the `path/to/istallation/folder` just defined and run:

```sh
  ./b2 install
```

This will leave Boost binaries in the lib/ subdirectory of your installation folder. You will also find a copy of the Boost headers in the include/ subdirectory of the installation prefix, so you can henceforth use that directory as an #include path.

### _Compiling the code_

After having cloned the repo and taken care of the dependencies as described above, it is necessary to define a `BOOST_ROOT` enviroment variable to point to the `path/to/istallation/folder` defined in the previous step on how to install Boost. This will be referenced in the [`CMakeLists.txt`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/CMakeLists.txt) file, lines 15-17:

```sh
  # REMEMBER to set BOOST_ROOT env variable to the root folder where Boost was installed
  set(BOOST_LIBRARYDIR $(BOOST_ROOT)/lib)
  set(BOOST_INCLUDEDIRS $(BOOST_ROOT)/include/boost)
```

Once done that, from the cloned repo you should just need to: 

1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make` 

### _Running  the code_

In order to run the code in [`main.cpp`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/src/main.cpp), from the `build` folder just type:

```sh
  ./pid 
```

This will output the log messages on the console and will also create a file with the same messages in the [logs](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/tree/log_and_test/logs) folder. 

### _Logger operation/setup_

The logger is configured through an external file called [`settings.txt`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/logs/settings.txt), in the [logs](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/tree/log_and_test/logs) folder. This implementation follows what documented [here](https://www.boost.org/doc/libs/1_72_0/libs/log/doc/html/log/detailed/utilities.html#log.detailed.utilities.setup.settings_file) and allows for changes in the logging structure (for example changes in the sinks, filtering or even enabling/disabling) without recompiling the code. At the moment both the console and the file sink are activated but they are configured with different severity levels, so to avoid flooding the terminal with messages. Anothere reference on how to build a logger using Boost (but without using the settings file) can be found [here](https://gernotklingler.com/blog/simple-customized-logger-based-boost-log-v2/). 

The log files are saved also in the [logs](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/tree/log_and_test/logs)  folder. The name of the file is built considering the date and time of the run, so to avoid overwritings.

Please note that there are several interesting features in the Boost logger that have not been implemented in this example for sake of simplicity but would be possible improvements for subsequent releases (for example setting limits to the number and size of the log files to save).

### _Test framework_

The Boost test framework is amongst those introduced and explained in the [CLion documentation](https://www.jetbrains.com/help/clion/unit-testing-tutorial.html#boost-test-framework) (CLion being my IDE of choice). The test suite of course can be used without having to rely on a specific IDE.

There are several other test frameworks available (most notably [Google test](https://github.com/google/googletest)) and this example does not aim at providing a comparison between all of them. Boost was selected for this project given the possibility to implement a Logger as described above, and because I believe it presents interesting capabilities for scientific computing given the broad scope of the libraries, but this should be considered just a personal opinion and choice. What is important is to recognize the value of a unit test framework, as a tool to increase quality and mainteinability of a piece of SW.

The code for the test suite can be found in the [`testPID.cpp`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/test/testPID.cpp) file, located in the [test](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/test/) folder. The folder also contains a [`CMakeLists.txt`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/test/CMakeLists.txt) file, to allow the building of an indipendent target in a separate folder.

In order to run the test suite, from the `build` folder you will just have to:

1. Change to the `test` folder that was created when compiling: `cd test`
2. Run `./pidTest`

A succesful run (no error detected in the code) would provide an output like this:

```sh
  ./pidTest
  Running 4 test cases...

  *** No errors detected
```
On the other hand, a failure in the test will be caught and shown with a specific error message, for example:

```sh
  ./pidTest
  Running 4 test cases...
  testPID.cpp:71: error: in "PIDTestSuite/PIDInitTest": Ki Init Failed
  testPID.cpp:148: error: in "PIDTestSuite/PIDErrorTest": Steering Calc 2 Failed

  *** 2 failures are detected in the test module "PIDTestModule"
```

More messages can be displayed by typing:

```sh
  ./pidTest --log_level=message
  Running 4 test cases...
  Setup Test Fixture
  Entering Init Test
  Leaving Init Test
  Teardown Test Fixture
  Setup Test Fixture
  Entering Set Test
  Leaving Set Test
  Teardown Test Fixture
  Setup Test Fixture
  Entering Error Calc Test
  Leaving Error Calc Test
  Teardown Test Fixture
  Setup Test Fixture
  Entering Tuning Test
  Tuning Test Step 1
  Tuning Test Step 2
  Leaving Tuning Test
  Teardown Test Fixture

  *** No errors detected
```

Please note that in the `logs` folder there is a specific settings file to be used by the test suite ([`settings_for_text.txt`](https://github.com/In-Progress-M-Russo/CarND-P8-PID-Control/blob/log_and_test/logs/settings_for_test.txt)). In it the logger is just disabled for the testing.
