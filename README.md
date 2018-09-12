# **Vehicle Control Using Model Predictive Controller "MPC"**
---

**In this project, I have worked on a Model Predictive controller "MPC" to control the vehicle's steering angle .**

## Project Introduction
---
In this project, we are controlling the vehicle's steering angle to be within the road using Model Predictive Controller "MPC".

## Model Discussion
---
In the following, I will be discussing the model used through the project

### What is Model Predictive Cpontroller "MPC" ? 
---
One approach to reach a good controller is to fisrt model the vehicle dynamics and constraints. This way we can analyse and tune the controller more efficiently.

The models describe mathematically how the vehicle moves. Some models are more realistic than others. Typically, more realistic models are more complex and more challenging to work with.

MPC reframes the control problem of following a trajectory as an optimization problem. The solution to the optimization problem is the optimal trajectory. MPC involves simulating different actuator inputs, predicting resulting trajectory, and selecting the trajectory with a minimum cost. We are constantly calculating inputs over a future horizon. That is why this approach is sometimes called **reducing horizon control**

### Model Definition
---
During this project, we will use a relatively simple type of model called the **kinematic model**

Kinematic models ignore many elements like gravity and tire forcesm but they are relatively simple to work with and they often perform well. Actually, kinematic models are simplification of dynamic models. Dynamic models aim to embody the actual vehicle dynamics as closely as possible. This might encompass tire forces, longitudinal and lateral forces, interia, gravity, air resistance, mass, ... etc

The used model within our project is defined in the following figure: 

![model](https://i.imgur.com/AMBb6nm.png)

- The vehicle is located at a positionwe track with `x` and `y`.
- Also, we need orientation which we denote with `psi`
- As the vehicle is moving, so it will have velocity `v`
- Inputs allow to control the state of the vehicle. Most cars have three actuators: steering, throttle `delta`, and brake peddals. For simplicity, we will consider the throttle and brake as a single actuator `a` where +ve values denote for accelerating and -ve value denote for braking.
- `Lf` measure the distance between the center of mass of the vehicle and its front axle.
- We want to minimize the predicated distance of the vehicle from the trajectory. That what we call **Cross Track Error** `cte`. In our case `cte` can be defined as the difference between the trajectory line and the current vehicle `y` position. Actually here we will not assume the reference trajectory as a line, *we will consider it as a third degree polynomial were most roads fit with it*.
- We want also to minimize the predicated differnece in angle between the vehicle orientation and the trajectory orientation. We call this psi error `epsi`
- `psides` is the desired orientation

### Cost Function
---
In order to develop and optimal controller, we need to define a cost function that captures the errors we want to minimize. The cost is actually the difference between where we want the vehicle to go and where it predicted it will go based on the model. Ideally for our cost function, we want to have both `cte` and `epsi` to be zero.

![cost](https://i.imgur.com/NKaW8xt.png)

### Prediction Horizon
---
The prediction horizon is the duration over which future prediction are made. This duration is defined by the product of  two parameters
- The number of time steps in the horizon `N`
- How much time elapses between actuations `dt`

`N` and `dt` are hyperparameters that I have tuned multiple times during my project development. The general rule for tuning these values is to have `dt` small as much as possible, with `N` that makes the prediction horizon as large as possible. The prediction horizon could be few seconds at most as beyond the prediction horizon the environment will change that it would not make sense to predict any further into the future. Also, MPC attempts to approximate a continuous referece trajectory by mean of the discrete paths between actuations. large values of `dt` result in loss of frequest actuaiotns which makes it harder to accurately approximate a continuous referece trajectory. This is sometimes called "discretization error".

So, I have chosen `dt = 0.1` and `N = 10` which results to a prediction horizon of `1 second` in the future. That resulted to an efficient trajectory prediction.

For targetting a real-life controller, in which the actuations do not apply immediately (not like simulation). So, a latency factor of 100 ms is applied. The purpose is to mimic real driving conditions where the car does actuate the commands instantly.
Dealing with latency is done by a thread that run every 100 ms. 

### Handling Latency
---
For targetting a real-life controller, in which the actuations do not apply immediately (not like simulation). So, a latency factor of 100 ms is applied. The purpose is to mimic real driving conditions where the car does actuate the commands instantly.
Dealing with latency is done by a thread that run every 100 ms. 
```cpp
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          this_thread::sleep_for(chrono::milliseconds(100));
```
Also, for taking this into account within the MPC, the actuations returned are the current actuations and the next ones after dt, so this make the trajectroy more efficient and stable and tackled the latency problem
```cpp
...
...
  return {solution.x[x_start + 1],   solution.x[y_start + 1],
          solution.x[psi_start + 1], solution.x[v_start + 1],
          solution.x[cte_start + 1], solution.x[epsi_start + 1],
          solution.x[delta_start] + solution.x[delta_start+1],   solution.x[a_start] +  solution.x[a_start+1]};
```


### Polynomial Fitting
---
We have mentioned in the model definition section that `cte` can be defined as the difference between the trajectory line and the current vehicle `y` position. Actually here we will not assume the reference trajectory as a line:
> **we use a third degree polynomial where most roads fit with it**.

The x and y points returned from the simulator are transformed to the vehicle coordinates as it simplifies the calculations


## Constraints
---
Physically, a vehicle can not make right angle turns. So, the steering angle is limited within -25 and +25 degrees which are -/+0.33 radians.

## Environment:
---
* Ubuntu 16.04 LTS
* Udacity Self-Driving Car Nano-Degree Term2 Simulator
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

## Running the Code
---
This project involves the Term 2 Simulator which can be downloaded [here](https://github.com/udacity/self-driving-car-sim/releases)

This repository includes two files that can be used to set up and install uWebSocketIO for either Linux or Mac systems. For windows you can use either Docker, VMware, or even Windows 10 Bash on Ubuntu to install uWebSocketIO.

Once the install for uWebSocketIO is complete, the main program can be built and ran by doing the following from the project top directory.

1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./mpc

## Conclusion
---
  * Model Predictive Controller "MPC" is one of the powerful and efficient tools that can be used in autonomous driving
  * MPC takes into consideration the state and the control input for the vehicle.
  * MPC model the vehcile dynamics and predicts in future how the vehicle will be.
  * MPC overcomes PID as MPC can apadt quite well because of the model, and taking the latency into consideration.

