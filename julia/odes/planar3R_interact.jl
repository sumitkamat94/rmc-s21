delete!(vis);
mvis, mechanism = display_urdf("planar3R.urdf",vis)
state = MechanismState(mechanism)

using Blink, Interact
mp = @manipulate for q1 in slider(-1.6:0.01:1.6; label="q1"), q2 in slider(-1.6:0.01:1.6; label="q2"), q3 in slider(-1.6:0.01:1.6; label="q3")
    set_configuration!(state,[q1;q2;q3])
    set_configuration!(mvis, configuration(state))
    # println(q1,q2)
    # hbox(vis)
end
# open(vis,Blink.Window())
w = Window()
body!(w,mp)

function setVisConfig(q,state,mvis)
    set_configuration!(state,q)
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
function calcOrinW(state,framenum)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[framenum])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,ee_frame,base_frame)
    or_in_ee = Point3D(ee_frame, 0.0, 0.0, 0.0);
    or_in_w = EEinBase*or_in_ee
    println("origin of frame $framenum in world: $or_in_w")
end
