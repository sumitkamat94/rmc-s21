# Assignment 2: Dynamics
**Due:** March 09, 2021

**What to submit:**
Provide the instructor with access to a (private or public) repository containing your (group's) solution.

Your solution (see problem description below) must contain:
1. A single file (pdf) that contains the complete derivation, derived by your group, of the Euler-Lagrangian dynamics of the mechanism. This pdf may be a scan of hand-written notes, but not camera photos of notes.
1. A single code file that provides a function named `custom_inversedynamics` that accepts three 4-element array inputs, and provides a 4-element array output.
1. A `README.md` file that lists the names of members of your group. If you are not using `Julia`, this file must also include a description of the software requirements to run your solution.

**Team Size:** No more than 3. Recommended: 2

## Problem Description:
For this assignment, you must derive the Euler-Lagrangian dynamics model for a four-link robot, and implement the inverse-dynamics solution (torque that generates a desired acceleration) in code. Assume that there are no external forces other than gravity, where $g = 9.81$m/s$^2$ acting on the robot. However:

**YOU CANNOT USE ANY DYNAMICS OR COORDINATE TRANSFORM PACKAGES**

You may use the package `LinearAlgebra`, apart from `Julia`'s base functions.

If you are not using Julia, the same limitation applies. You may at most use basic linear algebra functions in your code. You cannot use any pre-defined functions, for example, coordinate transformations or computing Jacobians. You will receive a score of zero if these functions are used.

### Robot Descriptions
The robot is described in the given URDF file `three_link_assn2.urdf`. Note that your solution code cannot load or parse this file, it is provided purely as part of the problem description. However, it will be useful for visualization and checking your solution.
