quat(theta,w) = [cos(theta/2),sin(theta/2)*w[1],sin(theta/2)*w[2],sin(theta/2)*w[3]]
function update_block_state(state,mvis,theta,w,d)
    w = w/norm(w);
    set_configuration!(state,vcat(quat(theta,w), d))
    set_configuration!(mvis,configuration(state))
end

delete!(vis)
mvis, mechanism = display_urdf("block.urdf",vis)
state = MechanismState(mechanism)
# frame rotation as axis, angle theta, w/norm(w)
theta = pi/3;
w = [1.0,1.0,1.0];
# frame origin in world
d = [0.0,0.0,0.0];
update_block_state(state,mvis,theta,w,d)

# Create base and end-effector frames:
ee_frame=default_frame(bodies(mechanism)[end])
base_frame = default_frame(bodies(mechanism)[1])


# Define point in frame of end effector
x = Point3D(ee_frame, 1.0, 0.0, 0.0)
# Visualize this point
setelement!(mvis,x, 0.05, "p1") # mvis, point, radius, name

# Obtain the transformation from world to EE
EEinBase = relative_transform(state,base_frame,ee_frame)
x_in_w = inv(EEinBase)*x
println("x in world: $x_in_w")
# x_in_ee = EEinBase * x_in_w

# println("x in block: $x_in_ee")
