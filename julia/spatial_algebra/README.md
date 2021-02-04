# Spatial Algebra
This folder contains code that enables exploration of the rigid body transformations, with the help of visualizations.

To start:
`include("startup.jl")`

To change the configuration of the three-link planar robot:
`q =  [1.0, 1.0,1.0] `
`set_configuration!(state,)`
`set_configuration!(mvis, configuration(state))`,
where `mvis` is the MechanismVisualizer, and the vector $q$ is joint variables.

The example in `main.jl`
* assigns a point to the end effector frame in the visualizer,
* calculates the position in the world frame, and
* and maps the result back into the end effector frame.
