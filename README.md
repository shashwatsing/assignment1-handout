# 16-362 Assignment 1: Quadrotor Dynamics and Control (Total: 100 points)

Goals: In this assignment, you will implement the mathematical model
of a quadrotor's dynamics as well as a position and attitude
controller to track simple trajectories.

### Academic Integrity
1. Do not publicly share your solution (using GitHub or otherwise)
2. Collaboration is encouraged but you should write final code on your own.

### 0.0 Setup
Create a python virtual environment.
```python
python3.8 -m venv .venv
```
Source the environment
```python
source .venv/bin/activate
```
You will need to install the following dependencies.
```python
pip install scipy pyyaml numpy matplotlib scikit-learn
```
Download the assignment.
```python
git clone git@github.com:mral-cmu/assignment1-handout.git
```
Sample data is provided at [TODO](). You may download
this data using the download script in the `data` directory.
```python
./download.sh
```

## 1.0 Rotations (20 points)
The directory containing rotation representations is in `rotation3.py`.
In this part of the assignment you will write the code to perform
the following conversions:

* Rotation matrix (3x3) -> Euler ZYX angles: `to_euler_zyx`
* Euler ZYX -> Rotation matrix (3x3): `from_euler_zyx`
* Rotation matrix (3x3) -> Quaternion (w,x,y,z): `to_quat`
* Quaternion (w,x,y,z) -> Rotation matrix (3x3): `from_quat`

All functions are contained within the rotation3.py file.
More information about each function follows.
Your code will be graded using Autolab. See Section 4 for
details about uploading and receiving scores for your
implementations.

### 1.1 `to_euler_zyx`
This function calculates the angles `phi=X`, `theta=Y`, `psi=Z` that
represent the rotation in the Z-Y-X Tait-Bryant
parameterization. The expected output is a 1x3 numpy array.  Note: the
expected output is in reverse order from functions like MATLAB's
`rotm2eul`; however, you can use this function to check your results.

### 1.2 `from_euler_zyx`
This function calculates the 3x3 rotation matrix from the input angles
`phi=X`, `theta=Y`, and `psi=Z`.

### 1.3 `roll`
This function extracts and returns the phi component from the
rotation matrix.

### 1.4 `pitch`
This function extracts and returns the theta component from the
rotation matrix.

### 1.5 `yaw`
This function extracts and returns the psi component from the
rotation matrix.

### 1.6 `from_quat`
This function calculates the 3x3 rotation matrix from a
(w,x,y,z)-parameterized quaternion.

### 1.7 `to_quat`
This function calculates the (w,x,y,z) quaternion from a 3x3 rotation
matrix.

## 1. Quadrotor Dynamics Simulator (50 points)
The quadrotor dynamics simulator is contained in
`quadrotor_simulator_py`. You will implement functions in this
folder, zip your folder, and upload to Autolab for grading.

Test data and expected solutions are available for local testing.

To receive full credit on this portion of the assignment,
you will need to implement the following four functions:

* `construct_mixer` (5 points)
* `calculate_world_frame_linear_acceleration` (5 points)
* `calculate_angular_acceleration` (5 points)
* `ode_step` (15 points)

### 1.1 `construct_mixer`
This function implements the mixer matrix as described in the lecture
slides.

### 1.1 `calculate_world_frame_linear_acceleration`
In this function you will implement Equation (4.2) from [1].

### 1.2 `calculate_angular_acceleration`
In this function you will implement Equation (4.3) from [1].

### 1.3 `ode_step`
This function implements the equations of motion for the quadrotor
dynamics model. The ODE solver is used to integrate the equations over
a period of time. The other two functions (1.1) and (1.2) will be
called in `ode_step`.

In this function, you will need to implement the following:
* Convert commanded RPMs (coming from the controller) to desired force and torques
* calculate the angular acceleration (see 1.3)
* calculate the linear acceleration (see 1.2)
* calculate the derivative of the quaternion using Equation (7) of [2].
* calculate the achieved RPMs

## 2. Position Controller (15 points)
You will need to write the following functions:

* `compute_body_z_accel`
* `compute_hod_refs`
* `compute_orientation`
* `compute_command`

Detailed instructions for the contents of each function follow:

### 2.1 `compute_body_z_accel`
This function uses the desired acceleration and current rotation to
calculate the body frame z acceleration.  See page 20 of [1] for
implementation details.

### 2.2 `compute_orientation`
This function calculates the desired orientation and takes the desired
acceleration and yaw reference as input.  Use Equations (33) -- (36)
from [4] to implement this function.

### 2.3 `compute_hod_refs`
This function uses the desired acceleration vector, flat reference,
and desired rotation to calculate the desired angular velocities and
accelerations.  Use Equations (14)--(25) of [3] to calculate the
angular velocities.  Use Equation (103)--(105) of [4] to calculate the
angular velocities. Disregard the drag component (i.e., set it to
zero) to make your life easier.

### 2.4 `compute_command`
This function contains the following functionality:
1. computes the PD feedback-control terms from the position and
   velocity control errors via Equation (32) of [3]
2. computes the desired rotation `compute_orientation`
3. applies the thrust command to the current body frame `compute_body_z_accel`
4. calculates the desired angular velocities and accelerations, which will
   be used in the inner control loop (detailed in the next section)

## 3. Attitude Controller (15 points)
You will need to write the following functions:

* `wrench_to_rotor_forces`
* `force_to_rpm`
* `run_ctrl`

### 3.1 `wrench_to_rotor_forces`
Uses the inverse of the mixer matrix to calculate rotor forces from
the thrust and torques.

### 3.2 `force_to_rpm`
Uses the forces to calculates the RPMs using the thrust coefficients.

### 3.3 `run_ctrl`
This function contains the following functionality:

1. calculates the error on orientation (see page 21 of [1])
2. calculates the angular velocity error (see page 21 of [1])
3. calculates the desired moments by pre-multiplying Equation 2.68 of
   [5] by the inertia matrix.
4. calculates the rotor forces using the `wrench_to_rotor_forces` function
5. calculates rpms from the rotor forces using the `force_to_rpm` function
6. calculates saturated rpms and returns

## 4. Grading with AutoLab
To have your solutions graded, you will need to tar the `quadrotor_simulator_py`
folder and upload to autolab.

```
cd assignment1-handout
tar -cvf handin.tar quadrotor_simulator_py
```

Autolab will run tests on each function you implement and you will
receive a score out of 100.  You may upload as many times as you like.
Note that we may regrade submissions after the deadline passes.

## References
[1] D. W. Mellinger, "Trajectory Generation and Control for Quadrotors" (2012). Publicly Accessible Penn Dissertations. 547. [https://repository.upenn.edu/edissertations/547](https://repository.upenn.edu/edissertations/547).

[2] E. Fresk and G. Nikolakopoulos, "Full quaternion based attitude control for a quadrotor," 2013 European Control Conference (ECC), Zurich, Switzerland, 2013, pp. 3864-3869, [doi: 10.23919/ECC.2013.6669617](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6669617).

[3] M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories." [https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf](https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf)

[4] M. Faessler, A. Franchi, and D. Scaramuzza, "Detailed Derivations of ``Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories''". [https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf](https://rpg.ifi.uzh.ch/docs/RAL18_Faessler.pdf)

[5] A. Spitzer, "Dynamical Model Learning and Inversion for Aggressive Quadrotor Flight". CMU-RI-TR-22-03. [https://www.ri.cmu.edu/app/uploads/2022/01/aspitzer_phd_ri_2022.pdf](https://www.ri.cmu.edu/app/uploads/2022/01/aspitzer_phd_ri_2022.pdf)
