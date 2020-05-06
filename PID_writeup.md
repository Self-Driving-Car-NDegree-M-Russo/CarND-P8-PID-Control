## PID Project

The purpose of this project is the implementation, in C++, of a PID controller for a vehicle driving around a virtual circuit.

**P**_roportional_ **I**_ntegral_ **D**_erivative_ (**PID**) controls are an [extremely well established technology](https://en.wikipedia.org/wiki/PID_controller), used in industrial control systems for decades. Even if the theory behind them relies on the ipothesis of linearity of the problem, they have been used in the most diverse use cases, thanks to the fact that they are simple and reliable.  The literature supporting them is wide and well known: in this writeup we will just give a  summary of the main idea behind them and we will show one of the many ways to tune such a controller.

The source code is contained in the [src](./src) folder in this git repo. It is the evolution of a starter project provided directly by Udacity, where two files where mainly modified: [`PID.cpp`](./src/PID.cpp) and [`PID.h`](./src/PID.h). The [`main.cpp`](./src/main.cpp) file has been left fundamentally unchanged, except for some specific handling of log messages.

The following sections of this writeup will provide details on the controller operations and the data flow, and in doing so the fundamental pieces of the code will be explained. A final [Results](PID_writeup.md#Results) section will show the behavior of the controller and describe possible next developements. 

---
## Data Input

The data source for this Filter will be the Udacity [simulator](https://github.com/udacity/self-driving-car-sim/releases). The compiled code will open a websocket session to the sim to read information about the state vehicles and provide back a steering command to it.

### Message Parsing

The parsing of the websocket message happens in [`main.cpp`](./src/main.cpp). The retrieval of information is clear in lines 81-83:

```sh
  double cte = std::stod(j[1]["cte"].get<string>());
  double speed = std::stod(j[1]["speed"].get<string>());
  double angle = std::stod(j[1]["steering_angle"].get<string>());
```
we can see that the message from the simulator includes the cross-track error of the vehicle (i.e. the error with respect to the reference trajectory), its current speed and steering angle. The cte will be an input for the subesequent control portion, and is also sent to the logger together with the steering angle, for debugging purposes (see lines 88-91):
 
 ```sh
  // DEBUG
  #ifdef PID_DEBUG
    std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
  #endif
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

All the previous actions are implemented in [`PID.cpp`](./src/PID.cpp), separated in two methods: `UpdateError` (lines 165-186) and `OutputSteeringAngle` (lines 188-197).

### _Initialization_

The controller is initialized through the `Init` method, in [`PID.cpp`](./src/PID.cpp), starting at line 11. In it the values of the three gains are set, as well as the respective errors and the requirement for tuining. The method is called from [`main.cpp`](./src/main.cpp), on line 64:

```sh
  // Initialize PID
  pid.Init(Kp, Ki, Kd, do_tune);
```

Note that the three gains are provided as input variables in the code and then, eventually, tuned, while the tuning flag is asked as a user's input through the code in lines 49-60:

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

The `UpdateError` method is called by [`main.cpp`](./src/main.cpp) at every message received from the simulator (see on line 87):

```sh
  pid.UpdateError(cte);
```
It takes in input the cross-track error as provided by the simulator and calculates (and assigns) the error terms for proportional/integral/derivative actions ([`PID.cpp`](./src/PID.cpp), lines 171-175):

```sh
  // NOTE: Previous cte is stored in previous p_error, and so the calculation of i_error must happen before the
  // update of p_error
  d_error = cte - p_error;
  p_error = cte;
  i_error += cte;
```

### _Computation of the Steering Angle_

The `OutputSteeringAngle` method is called by [`main.cpp`](./src/main.cpp) just after the `UpdateError` one (line 88):

```sh
  steer_value = pid.OutputSteeringAngle();
```

And the implementation can be found in [`PID.cpp`](./src/main.cpp) on line 194:

```sh
  double steering = -Kp * p_error - Ki * i_error -Kd * d_error;
```

The steering angle so calculated s then built in a message passed back to the sim at every iteration ([`main.cpp`](./src/main.cpp), lines 100-103):

```sh
  json msgJson;
  msgJson["steering_angle"] = steer_value;
  msgJson["throttle"] = 0.3;
  auto msg = "42[\"steer\"," + msgJson.dump() + "]";
```

### _PID Tuning_

As mentioned in the first section, PID controller are the subject of an extremely extensive bibliography, covering design, implementation and tuning (from [here](https://www.academia.edu/27771107/PID_Controllers_2nd_Edition_%C3%85str%C3%B6m_Karl_J._H%C3%A4gglund_Tore_) and [here](https://www.amazon.com/Automatic-Tuning-Controllers-Karl-Astrom/dp/1556170815) to [here](https://www.intechopen.com/books/pid-control-for-industrial-processes/advanced-methods-of-pid-controller-tuning-for-specified-performance) and much, much more).

In this project we have applied the Coordinate Ascent (_Twiddle_) method as described in the Udacity lecture available [here](https://youtu.be/2uQ2BSzDvXs). Using this method the three controller's gains are recursively perurbed and the effect of that perturbation on the performance of the system is measured. The space of the controller gains is explored in the direction of improved performances, until the perturbations required to further progress become negligible.

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

The implementation of the method can be found in the `TuneGains` method in [`PID.cpp`](./src/PID.cpp), (lines 199-301). Few notes:

* The implementation had to be modified to take into account the fact that the tuning algorithm had to run in parallel with the simulator, that is providing the evaluation of the cross-track error in real time. Hence it was necessary to introduce some flags to keep track of the previous state in the cycle. The method is called at every measurement's update in [`main.cpp`](./src/main.cpp), lines 95-98:

```sh
  // If tuning flag active, call tuning algorithm
  if (pid.GetTuneFlag()){
    pid.TuneGains();
  }
```

* As a measurement of the change in performances, I am using the square root of the squared error, calculated as part of the `UpdateError` method ([`PID.cpp`](./src/PID.cpp), line 177):

```sh
  s_error = sqrt(pow(cte,2));
```

* The tuning process start after a given number of iterations (by default 350). This allows the vehicle to reach some speed before starting to affect its trajectory.
* The tuning process is stopped when the average delta gain, in percentage, becomes lower than a given threshold. This parameter is defined in [`PID.cpp`](./src/PID.cpp), line 210:

```sh
  double dp_avg = (fabs(dp[0]/p[0]) + fabs(dp[1]/p[1]) + fabs(dp[2]/p[2])) / 3.0;
```

* The tuning cycle is constrained to not exceed a given number of iterations (by default 1000).

---
## Results

As mentioned in the sections above, some values for the controller's gains are provided in input, and the user can decide whether or not to activate the tuning on it.

The initial values provided have been identified through some manual exercise (varying the proportional controller alone first, then operating on the derivative term and finally on the integral) as well as considering some available references from similar projects from other students (see [here](https://github.com/wlsmith42/PID-Control) and [here](https://medium.com/intro-to-artificial-intelligence/pid-controller-udacitys-self-driving-car-nanodegree-c4fd15bdc981) for example). The final choice is:

_Gain_ | _Value_
---- | :----:
`Kp` | 0.1
`Ki` | 0.01
`Kd` | 2.5

These values are assigned in [`main.cpp`](./src/main.cpp), lines 69-73:

```sh
  // Set PID gains
  double Kp = 0.1;      // Initial value for Kp
  double Ki = 0.001;    // Initial value for Ki
  double Kd = 2.5;      // Initial value for Kd
```

And an recording of the track followed with these gains is visible in the video here below:

[![Default Run](http://img.youtube.com/vi/SpYnDDWD634/0.jpg)](https://www.youtube.com/watch?v=SpYnDDWD634 "Default Run")

As it can be seen these gains are effective in finishing the track, but there could be room for improvement. Indeed, if we activate the tuning option from command line when we run the code we can see that the tuning ends with gains like this:

_Gain_ | _Value_
---- | :----:
`Kp` | 0.14
`Ki` | 0.116
`Kd` | 2.9

**NOTE**: About these values, see also the final section on further improvements.

The track recorded in that case is the following:

[![Tuning Run](http://img.youtube.com/vi/G1QeqORZkuI/0.jpg)](https://https://www.youtube.com/watch?v=G1QeqORZkuI "Tuning Run")

### _Difference between Debug and Relase code_

As explained in the [`README`](./README.md#compiling-the-code) file, this code can be built and run in either a `Debug` or `Release` mode, the difference being the kind of information displayed to the user on the console. 

In the case of a `Release` run, for example, the output would be something like:

```sh
  Listening to port 4567
  Connected!!!
  =========================
  Tuning threshold crossed:
  Tuned parameters:
  Kp = 0.181445 Ki = 0.00149228 Kd = 3.72024
  =========================
```

While for a `Debug` run we would see much more stuff:

```sh
  Listening to port 4567
  Connected!!!
  *************************
  Iteration : 1
  CTE: 0.7598 Steering Value: -1.97624
  42["steer",{"steering_angle":-1.9762398,"throttle":0.3}]
  *************************
  Iteration : 2
  CTE: 0.7598 Steering Value: -0.0774996
  42["steer",{"steering_angle":-0.0774996,"throttle":0.3}]
  *************************

  ...
  
  *************************
  Iteration : 418
  CTE: 0.293 Steering Value: -0.11346
  -------------------------
  Average dp/p = 0.09 against threshold : 0.01
  -------------------------
  TUNING
  Current p index : 2
  Cycle start - Increment p[p_it] by dp[p_it]
  -------------------------
  Adjusted parameters ...
  Kp = 0.259374 Ki = 0.00259374 Kd = 6.42541
  dKp = 0.0233437 dKi = 0.000233437 Kd = 0.530538
  Error = 0.293
  42["steer",{"steering_angle":-0.113460316482461,"throttle":0.3}]
  *************************
  Iteration : 419
  CTE: 0.3154 Steering Value: -0.124899
  -------------------------
  Average dp/p = 0.0875229 against threshold : 0.01
  -------------------------
  TUNING
  Current p index : 2
  Case number two: increment p executed but NO best error found. Decrement p[it] by 2*dp[p_it]
  -------------------------
  Adjusted parameters ...
  Kp = 0.259374 Ki = 0.00259374 Kd = 5.36433
  dKp = 0.0233437 dKi = 0.000233437 Kd = 0.530538
  Error = 0.3154
  42["steer",{"steering_angle":-0.124898579254641,"throttle":0.3}]
  *************************
  
  ...
  
  *************************
  Iteration : 565
  CTE: -0.0487 Steering Value: -0.177524
  =========================
  Tuning threshold crossed:
  Tuned parameters:
  Kp = 0.130801 Ki = 0.00121 Kd = 3.29725
  =========================
  -------------------------
  Adjusted parameters ...
  Kp = 0.130801 Ki = 0.00121 Kd = 3.29725
  dKp = 0.00131073 dKi = 1.07242e-05 Kd = 0.0364092
  Error = 0.0487
  42["steer",{"steering_angle":-0.1775243393,"throttle":0.3}]
  *************************
  Iteration : 566
  CTE: -0.0126 Steering Value: -0.151896
  42["steer",{"steering_angle":-0.1518963094,"throttle":0.3}]
  *************************
  
  ...
  
```

In order to change the content/format of the messages, the code will have to be modified and recompiled.

### _Final considerations and next steps_

* The behavior of the vehicle can still be improved: amongst ideas that could be evaluated we can mention a different performance indicator to consider (for example averaging or comunlating a few measurements of the cross-track error rather than a single one) or the definition of a different threshold to allow (for example) a longer tuning.
* I noticed that if we run the code twice we would end up with _similar_ but not _identical_ values for the tuned gains. I believe that this is a consequence of the the non-linearity of the model likely implemented by the simulator, that affects the behavior of the tuning algorithm.
* Indeed a tuning of the PID against a linear/linearized model of the vehicle would probably provide the best option for a robust identification of the gains: this could still be enhanced using the tuning algorithm against the simulator.
