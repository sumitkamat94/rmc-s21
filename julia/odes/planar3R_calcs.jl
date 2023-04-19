## Run once:
# delete!(vis);
# mvis, mechanism = display_urdf("planar3R.urdf",vis)
# state = MechanismState(mechanism)
# setVisConfig([0,0,pi/2],state,mvis)

## Extract the end-effector frame 
ee_frame=default_frame(bodies(mechanism)[end])
## Extract the base-effector frame 
base_frame = default_frame(bodies(mechanism)[1])
## Define point in frame of end effector
x_in_EE = Point3D(ee_frame, 0.0, 0.0, 0.0)
## Extract the relative transformation between frames given the current  state
# rigid body transform from at current state ee to base 
@show EEinBase = relative_transform(state,ee_frame,base_frame)
@show rotation(EEinBase)
@show translation(EEinBase)
@show x_in_EE
@show EEinBase*x_in_EE