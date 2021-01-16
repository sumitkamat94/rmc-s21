## Kalman Filtering and SLAM

This folder contains an implementation of a Kalman Filter, and its extension (Extended Kalman Filter) used to solve Simultaneous Localization and Mapping (SLAM) problems.

### Instructions:
* After entering the `read-eval-print loop` (REPL), setup the environment by running `include("startup.jl")`
* Execute either: `include("RRT.jl")`
  * 1D Mass Particle: `include("kf_point_mass_1D.jl")`
  * Translating point in plane: `include("slam_simple_point.jl")`
  * Kinematic Differential Drive Robot: `include("slam_rotating_point.jl")`
* Note: `run_ddr.jl` is unfinished
