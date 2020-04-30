## PID Project

The purpose of this project is the implementation, in C++, of a PID controller for a vehicle driving around a virtual circuit.

**P**_roportional_ **I**_ntegral_ **D**_erivative_ (**PID**) controls are an [extremely well established technology](https://en.wikipedia.org/wiki/PID_controller), used in industrial control systems for decades. Even if the theory behind them relies on the ipothesis of linear system, they have been used in the most diverse use cases, thanks to the fact that they are simple and reliable.  The literature supporting them is wide and well known: in this writeup we will just give a  summary of the main idea behind them and we will show one of the many ways to tune such a controller.

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where two files where mainly modified: [`PID.cpp`](./src/PID.cpp) and [`PID.h`](./src/PID.h). The [`main.cpp`](./src/main.cpp) file has been left fundamentally unchanged, except for the implementation of a logging capability.

The following sections of this writeup will provide details on the controller operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final [Results](PID_writeup.md#PID-results) section will show the outcomes of the filter running against two different data sets. 

---
## Data Input

The data source for this Filter will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases). The compiled code will open a websocket session to the sim to read information about the state vehicles and provide back a steering command to it.

### Message Parsing

The parsing of the websocket message happens in `main.cpp`. The retrieval of information is clear in lines 110-113:

```sh
  double cte = std::stod(j[1]["cte"].get<string>());
  double speed = std::stod(j[1]["speed"].get<string>());
  double angle = std::stod(j[1]["steering_angle"].get<string>());
```
 we can see that the message from the simulator includes the cross-track error of the vehicle (i.e. the error with respect to the reference trajectory), its current speed and steering angle. cte will be used by the subesequent control portion, and is also sent to the logger together with the steering angle, for debugging purposes (see lines 119-120):
 
 ```sh
  // DEBUG
  BOOST_LOG_TRIVIAL(debug) << "CTE: " << cte << " Steering Value: " << steer_value;
 ```
---
## Implementation of the Controller

As the name says, a PID controller is composed by 3 main parts:

_Control_ | _Definition_
---- | ----
**P** Control | Proportional controller: provides an output (steering action) directly proportional to the cross-track error
**D** Control | Derivative controller: provides an output directly proportional to the _time derivative_ of the cross-track error. This helps preventing overshoots that would rise with the Proporional action only
**I** Control | Integral controller: provides an output directly proportional to the _integral (on time)_ of the cross-track error. This eliminates the effects of biases that would affect a pure PD controller, and allows a steady state error = 0.


---
## PID_results

