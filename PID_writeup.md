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
**P** Control | Proportional controller: provides an output (steering action) directly proportional to the cross-track error.
**D** Control | Derivative controller: provides an output directly proportional to the _time derivative_ of the cross-track error. This helps preventing overshoots that would rise with the Proporional action only.
**I** Control | Integral controller: provides an output directly proportional to the _integral (on time)_ of the cross-track error. This eliminates the effects of biases that would affect a pure PD controller, and allows a steady state error = 0.

The proportionality coefficients for each of the actions are often referred to as the controller's _Gains_.

All the previous actions are implemented in [`PID.cpp`](./src/PID.cpp), separated in two methods: `UpdateError` (lines 173-191) and `OutputSteeringAngle` (lines 193-202).

### _Initialization_

The controller is initialized through the `Init` method, in [`PID.cpp`](./src/PID.cpp), starting at line 19. In it the values of the three gains are set, as well as the respective errors and the requirement for tuining. The method is called from [`main.cpp`](./src/main.cpp), on line 93:

```sh
  // Initialize PID
  pid.Init(Kp, Ki, Kd, do_tune);
```

Note that the three gains are provided as input variables in the code and then, eventually, tuned, while the funing flags is asked as a user's input through the code in lines 78-89:

```sh
  //Set tuning flag
  bool do_tune = false;
  string do_tune_in;
  std::cout <<"Do you want to enable tuning with Coordinated Ascent (Twiddle) method [y/(n)]? ";
  getline(std::cin, do_tune_in);
  if ((do_tune_in.compare("Y") == 0) || (do_tune_in.compare("y") == 0)){
    std::cout << "Tuning enabled" <<std::endl;
    do_tune = true;
  }
  else {
    std::cout << "Tuning NOT enabled" <<std::endl;
  }
```

By default tuning is not enabled.

### _Errors' Update_

The `UpdateError` method is called by [`main.cpp`](./src/main.cpp) at every message received from the simulator (see on line 116):

```sh
  pid.UpdateError(cte);
```
It takes in input the cross-track error as provided by the simulator and calculates (and assigns) the error terms for proportional/integral/derivative actions ([`PID.cpp`](./src/PID.cpp), lines 179-183):

```sh
  // NOTE: Previous cte is stored in previous p_error, and so the calculation of i_error must happen before the
  // update of p_error
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
```

### _Computation of the Steering Angle_

The `OutputSteeringAngle` methos is called by [`main.cpp`](./src/main.cpp) just after the `UpdateError` one (line 117):

```sh
  steer_value = pid.OutputSteeringAngle();
```

And the implementation can be found in [`PID.cpp`](./src/main.cpp) on line 199:

```sh
  double steering = -Kp * p_error - Ki * i_error -Kd * d_error;
```

The steering angle so calculated s then built in a message passed back to the sim at every iteration ([`main.cpp`](./src/main.cpp), lines 127-130):

```sh
  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = 0.3;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
```

### _PID Tuning_

As mentioned in the first section, PID controller are the subject of an extremely extensive bibliography, covering design, implementation and tuning (from [here](https://www.academia.edu/27771107/PID_Controllers_2nd_Edition_%C3%85str%C3%B6m_Karl_J._H%C3%A4gglund_Tore_) and [here](https://www.amazon.com/Automatic-Tuning-Controllers-Karl-Astrom/dp/1556170815) to [here](https://www.intechopen.com/books/pid-control-for-industrial-processes/advanced-methods-of-pid-controller-tuning-for-specified-performance) and much, much more).

In this project we have applied the Coordinate Ascent (_Twiddle_) method as described in the Udacity lecture available [here](https://youtu.be/2uQ2BSzDvXs). Using this method the three controller gains are recursively perurbed and the effect of that perturbation on the performance of the system is measured. The space of the controller gains is explored in the direction of improved performances, until the perturbations required to further progress become negligible.

Some pseudo-code for the alogorithm would look like:

```sh
  for i in range(len_vect_gains)
    P[i] += dP[i]
    calculate_err
    if err < best_err
      best_err = err
      dP[i] *= 1.1
    else
      P[i] -= 2*dP[i]
      calculate_err
      if err < best_err
        best_err = err
        dP[i] *= 1.1
      else
        P[i] += dP[i]
        dP[i] *= 0.9
```

The implementation of the method can be found in the `TuneGains` method in [`PID.cpp`](./src/PID.cpp), (lines 204-284). Few notes:

* The implementation had to be modified to take into account the fact that the tuning algorithm had to run in parallel with the simulator, that is providing the evaluation of the cross-track error in real time. Hence it was necessary to introduce some flags to keep track of the previous state in the cycle. The method is called at every measurement's update in [`main.cpp`](./src/main.cpp), lines 122-125:

```sh
  // If tuning flag active, call tuning algorithm
  if (pid.GetTuneFlag()){
    pid.TuneGains();
  }
```

* As a measurement of the change in performances, I am using the square root of the squared error, calculated as part of the `UpdateError` method ([`PID.cpp`](./srd/PID.cpp), line 185):

```sh
  s_error = sqrt(pow(cte,2));
```

* The tuning process is stopped when the average delta gain, in percentage, becomes lower than a given threshold. This parameter is defined in [`PID.cpp`](./src/PID.cpp), line 215:

```sh
  double dp_avg = (fabs(dp[0]/p[0]) + fabs(dp[1]/p[1]) + fabs(dp[2]/p[2])) / 3.0;
```

---
## PID results


