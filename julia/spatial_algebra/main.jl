function update_planar_state(state,mvis,q1,q2,q3)
    set_configuration!(state, [q1, q2,q3])
    set_configuration!(mvis, configuration(state))
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

    or_w = Point3D(ee_frame, 0.0, 0.0, 0.0);
    x1_w = inv(EEinBase)*(Point3D(ee_frame, 1.0, 0.0, 0.0) - or_w)
    x2_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 1.0, 0.0) - or_w)
    x3_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 0.0, 1.0) - or_w)
    R = hcat(x1_w.v,x2_w.v,x3_w.v)
    x_in_w_manual = R *x.v + (inv(EEinBase)*Point3D(ee_frame, 0.0, 0.0, 0.0)).v
    println("x in world manual: $x_in_w_manual")
end

# Load mechanism and visualize at some config
delete!(vis)
mvis, mechanism = display_urdf("planar3R.urdf",vis)
state = MechanismState(mechanism)
update_planar_state(state,mvis,pi/3,pi/3,pi/3)

# Create base and end-effector frames:
ee_frame=default_frame(bodies(mechanism)[end])
base_frame = default_frame(bodies(mechanism)[1])
# Define point in frame of end effector
x = Point3D(ee_frame, 1.0, 0.0, 0.0)
# Visualize this point
setelement!(mvis,x, 0.05, "p1") # mvis, point, radius, name
calcXinW(state,x)
