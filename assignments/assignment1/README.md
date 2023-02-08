# Assignment 1: Kinematics
**Due:** February 17 11:59 pm
**What to submit:** Provide the instructor with access to a (private or public) repository containing your (group's) solution. You must also create a description of the software requirements to run your example, and the names of the functions that embody your solution and the arguments they require. Also include names of members of the group.
**Team Size:** No more than 3. Recommended: 2
### Description:
For this assignment, you must provide code that solves inverse kinematics problems and visualizes the start and end configurations of your robotic manipulator. BONUS: Animate the motion from start to end configurations. 

1. Define a custom URDF for a robotic manipulator with a kinematic structure that is open, has at least 10 degrees of freedom, and is not planar. The tree must have at least one branch point, and your code should automatically handle cases where the user provides some or all end-effector points. 
1. An end-effector corresponds to a leaf node in the kinematic tree. Create a function that takes start and end coordinates in the world frame for a point attached to (one of) the end effector frame(s) and animates the corresponding motion trajectory. 
    * If necessary, provide a constructor for each of these points that accepts three floating point numbers.

If you use `Julia` for this assignment, you may use the `startup.jl` file and the environment files from the `odes` folder.
