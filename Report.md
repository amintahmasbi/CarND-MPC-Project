# CarND-Controls-MPC
MPC controller project

---
## Reflections
### The Model
I developed and used the __Kinematic model__ explained in the course:

1. $$x_{t+1} = x_t + v_t * cos(\psi_t) * dt$$
2. $$y_{t+1} = y_t + v_t * sin(\psi_t) * dt$$
3. $$\psi_{t+1} = \psi_t + \frac {v_t} { L_f} * \delta * dt$$
4. $$v_{t+1} = v_t + a_t * dt$$ 
5. $$cte_{​t+1}​​ = f(x_{t}) − y_{​t}  ​​ +(v_{​t} ​​∗ sin(eψ_{​t}) ∗ dt )$$
6. $$e\psi_{t+1} = \psi_t  - \psi{des}_t  + (\frac{v_t} { L_f} * \delta_t * dt)$$

 Where $[x, y, \psi, v,cte,eψ]$ is the state of the vehicle, ${L_f}$ is a physical characteristic of the vehicle, $[cte,e\psi] $ are the errors, and $[δ,a]$ are the actuators, or control inputs, to the system.
 ---
### Timestep Length and Elapsed Duration (_N & dt_)
The prediction horizon (_T_) is the duration over which future predictions are made. _T_ is the product of two other variables, _N_ and _dt_.

_N_ is the number of timesteps in the horizon. _dt_ is how much time elapses between actuations. For example, if _N_ were 20 and _dt_ were 0.5, then _T_ would be 10 seconds.

_N, dt, and T_ are hyperparameters to tune for each model predictive controller. However, there are some general guidelines. 
_T_ should be as large as possible, while _dt_ should be as small as possible. These guidelines create tradeoffs.

___Horizon:___
In the case of driving a car, _T_ should be a few seconds, at most. Beyond that horizon, the environment will change enough that it won't make sense to predict any further into the future.

For this project, the best horizon would be the one that streches to all the way to the last available waypoint (6th waypoint), which depends on the desired target velocity. I used a horizon of 2 seconds (`T = 2 sec`) with the target speed of _40 mph_.


___Number of Timesteps:___
The goal of Model Predictive Control is to optimize the control inputs: $[δ,a]$. An optimizer will tune these inputs until a low cost vector of control inputs is found. The length of this vector is determined by _N_. Thus _N_ determines the number of variables the optimized by MPC. This is also the major driver of computational cost.

For this project, based on the Horizon and Timestep duration, the value of `N = 10`, which is good enough for the optimizer to find the path.

___Timestep Duration:___
MPC attempts to approximate a continues reference trajectory by means of discrete paths between actuations. Larger values of _dt_ result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory. This is sometimes called "discretization error".

A good approach to setting _N, dt, and T_ is to first determine a reasonable range for T and then tune dt and N appropriately, keeping the effect of each in mind. 

For this project, the best amount of _dt_ would be the average of control processing delay, which makes the updates of control signal synchronized with the MPC prediction steps. Therefore, if there was no delay, `dt = 0.1` (_100ms_) would be a good value, so with delay `dt = 0.2` (_200ms_) gives an stable result.

---
### Polynomial Fitting and MPC Preprocessing

1. All the waypoints were transformed to the vehicle coordination system. This transformation makes the error calculation easier and also assures that there will be no two points with same _x_ value to derive a function for them.

2. I used `polyfit` function from the course content to fit a _3rd_ order polynomial to the given _x_ and _y_ coordinates representing waypoints. 

3. Finally, I used `polyeval` and `polyPrimEval` to evaluate _y_ and _yaw_ values of given _x_ coordinates, respectively.

---
### Model Predictive Control with Latency
A contributing factor to latency is actuator dynamics, the time elapsed between when you command a steering angle to when that angle is actually achieved. This could easily be modeled by a dynamic system and incorporated into the vehicle model.
 
For this project, I used the above mentioned __Kinematic model__ to model the vehicle movement starting from the current state for the duration of the latency. The resulting state from it (called `next_state`) is passed to the MPC as the new initial state.
***Note:***
I divided the prediction into smaller time steps (_5ms_).That way the current steering angle would iteratively change the yaw angle.






















