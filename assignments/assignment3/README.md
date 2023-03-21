# Assignment 2: Control
**Due:** March 27, 2023

**Individual Assignment**

**What to submit:**
Provide the instructor with access to a public repository containing your solution.

Your solution must contain files that implement the functions in the problem description. If you are not using `Julia`, include a `README.md` file that describes how to run your solution and simulate control of the Franka Emika panda as defined in `panda.urdf`

**Team Size:** 1

## Problem Description:
There are three parts to this assignment:
1. Define a joint angle trajectory for the Panda robot joint angle that begins at the starting configuration $q_0$ at time $0$ sec and ends at the desired configuration $q_d$ at time $10$ sec. Implement this trajectory as a function `traj` that has input as a real number (time $t$) and output as the desired joint angle vector $q_d(t)$ at time $t$ and its first two time-derivatives $\dot q_d(t)$ and $\ddot q_d(t)$ (see the file `solution_template.jl`).
`q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1]`
`qd = [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]`
1. Define a function `control_PD!` that implements a PD trajectory-tracking controller. This controller must achieve a final configuration error whose norm is less than $0.01$, and where the torque is saturated at $\pm 50$ Nm. (See the file `solution_template.jl`.)
1. Define a function `control_CTC!` that implements a computed torque controller. Use a PD and feedforward controller for the resulting linear system. This controller must achieve a final configuration error whose norm is less than $0.01$, and where the torque is saturated at $\pm 50$ Nm. (See the file `solution_template.jl`.) **How small can you make the proportional gains in this controller relative to the PD controller?**
