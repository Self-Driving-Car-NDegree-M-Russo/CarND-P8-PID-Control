## PID Project

The purpose of this project is the implementation, in C++, of a PID controller for a vehicle driving around a virtual circuit.

**P**_roportional_ **I**_ntegral_ **D**_erivative_ (**PID**) controls are an [extremely well established technology](https://en.wikipedia.org/wiki/PID_controller), used in industrial control systems for decades. Even if the theory behind them relies on the ipothesis of linear system, they have been used in the most diverse use cases, thanks to the fact that they are simple and reliable.  The literature supporting them is wide and well known: in this writeup we will just give a  summary of the main idea behind them and we will show one of the many ways to tune such a controller.

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where two files where mainly modified: [`PID.cpp`](./src/PID.cpp) and [`PID.h`](./src/PID.h). The [`main.cpp`](./src/main.cpp) file has been left fundamentally unchanged, except for the implementation of a logging capability.

The following sections of this writeup will provide details on the controller operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final [Results](PID_writeup.md#PID-results) section will show the outcomes of the filter running against two different data sets. 

---
## Data Input

---
## PID_results

