# Assignment 1: Kinematics
**Due:** February 16
**What to submit:** Provide the instructor with access to a (private or public) repository containing your (group's) solution. You must also create a description of the software requirements to run your example, and the names of the functions that embody your solution. Also include names of members of the group.
**Team Size:** No more than 3. Recommended: 2
### Description:
For this assignment, you must provide code that solves inverse kinematics problems, simulates the result, and animates the simulation.

1. Define a custom URDF for a robotic manipulator with a kinematic structure that is open, has at least 10 degrees of freedom, and is not planar.
    * **ME699:** The tree must have at least one branch point, and your code should automatically handle cases where the user provides some or all end-effector points.
1. An end-effector corresponds to a leaf node in the kinematic tree. Create a function that takes start and end coordinates in the world frame for a point attached to (one of) the end effector frame(s) and animates the corresponding motion trajectory.
    * If necessary, provide a constructor for these points that accepts three floating point numbers.

If you use `Julia` for this assignment, you may use the `startup.jl` file and the environment files from the `odes` folder.
