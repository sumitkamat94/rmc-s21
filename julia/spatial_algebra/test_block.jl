quat(theta,w) = [cos(theta/2),sin(theta/2)*w[1],sin(theta/2)*w[2],sin(theta/2)*w[3]]
function update_block_state(state,mvis,theta,w,d)
    w = w/norm(w);
    set_configuration!(state,vcat(quat(theta,w), d))
    set_configuration!(mvis,configuration(state))
end

function calcXinW(state,x)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[end])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,base_frame,ee_frame)
    x_in_w = inv(EEinBase)*x
    println("x in world: $((x_in_w).v)")
    # x_in_ee = EEinBase * x_in_w
    # println("x in block: $x_in_ee")

    # Manual reconstruction:
    or_w = Point3D(ee_frame, 0.0, 0.0, 0.0);
    x1_w = inv(EEinBase)*(Point3D(ee_frame, 1.0, 0.0, 0.0) - or_w)
    x2_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 1.0, 0.0) - or_w)
    x3_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 0.0, 1.0) - or_w)
    R = hcat(x1_w.v,x2_w.v,x3_w.v)
    x_in_w_manual = R *x.v + (inv(EEinBase)*Point3D(ee_frame, 0.0, 0.0, 0.0)).v
    println("x in world manual: $x_in_w_manual")
end
delete!(vis)
mvis, mechanism = display_urdf("block.urdf",vis)
state = MechanismState(mechanism)
# frame rotation as axis, angle theta, w/norm(w)
theta = 0.0;
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


#update_block_state(state,mvis,pi/3,w,d)
#calcXinW(state,x)
