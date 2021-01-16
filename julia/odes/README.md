## Simulation using Julia

This folder contains a examples of simulations systems modeled as Ordinary Differential Equations (ODEs). These ODEs may be explicitly provided to the simulator, such as in `lorenz.jl` or `pendulum.jl`. These explicit equations, -- code corresponding to symbolic equations -- may be integrated by any suitable scheme. For robotic mechanisms, we describe the links and joints, and let a computer program use that information to integrate the ODEs implicitly. A symbolic ODE is not required for simulation.

### Instructions:
* After entering the `read-eval-print loop` (REPL), setup the environment by running `include("startup.jl")`
* You can simulate any system by includeing the corresponding `.jl` file.
* When a URDF is given, we are able to visualize the solution in 3D using `MeshCat` and `MeshCatMechanisms`
* If no URDF exists, we instead plot the trajectories 
