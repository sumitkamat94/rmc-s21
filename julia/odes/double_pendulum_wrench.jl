function jacs()
    set_configuration!(state,rand(2))
    jacs(state)
end
function jacs(state)
    mechanism = state.mechanism
    # We may manually construct the Geom Jacobian as defined in RMC book
    #  using the joint axes and origins, where all are in base frame.
    frame1  = default_frame(bodies(mechanism)[3])
    frame2  = default_frame(bodies(mechanism)[4])
    frame3  = default_frame(bodies(mechanism)[5])
    o1 = relative_transform(state,frame1,root_frame(mechanism))* Point3D(frame1,0.0,0.0,0.0)
    o2 = relative_transform(state,frame2,root_frame(mechanism))* Point3D(frame2,0.0,0.0,0.0)
    oee = relative_transform(state,frame3,root_frame(mechanism))* Point3D(frame3,0.0,0.0,0.0)
    Jv1 = LinearAlgebra.cross([0,1,0],Array((oee-o1).v));
    Jv2 = LinearAlgebra.cross([0,1,0],Array((oee-o2).v),);
    Jq_lin_man = hcat(Jv1, Jv2 )

    # Most algorithms use the Jacobians defined in the Modern Robotics book
    # Spatial Jacobian of end-effector in base frame
    mech_path = path(mechanism,root_body(mechanism),bodies(mechanism)[end])
    Js = geometric_jacobian(state,mech_path)
    # Body Jacobian  (end-effector)
    # Jb = Ad_{T_{sb}} Jq,
     # Manually, Jb.ang = R_bs Js.ang; Jb.lin = [p_bs] R_bs Js.ang + R_bs Js.lin
     # One version of transform() implements this:
    Jb=transform(Js,relative_transform(state,root_frame(mechanism),default_frame(bodies(mechanism)[end])))
    # Neither Jq nor Jqb are appropriate for a Force in coordinates aligned with the spatial frame
    # We need the Geometric Jacobian Jq as defined in RMC (Spong,Vidyasagar,Hutchinson)
    # Fortunately, we don't need to construct it manually
    # We use the transformation T_{bs} to do so (T_{bs} maps spatial coords to body coords)
    T_bs = relative_transform(state,root_frame(mechanism),default_frame(bodies(mechanism)[end]))
    R_bs = rotation(T_bs);
    p_bs = translation(T_bs); #-> - Rt'Ts
    hat_p_bs = [ 0 -p_bs[3] p_bs[2]; p_bs[3] 0 -p_bs[1]; -p_bs[2] p_bs[1] 0];
    # We may derive Jq.linear from Js using T_bs:
    Jq_lin = Js.linear + R_bs'* hat_p_bs*R_bs * Js.angular

    println("Linear part of Spatial Jacobian Jq:  ", Js.linear)
    println("Linear part of Body Jacobian Jq:  ", Jb.linear)
    println("Linear part of Geometric Jacobian (RMC):  ",Jq_lin )
    println("Manually constructed Geometric Jacobian:",Jq_lin_man)

end
function control!(τ, t, state)
    # Add some damping
    τ .= - 20*velocity(state)

    # Add a P control  damping
    τ .+= 100*(qd-configuration(state))
    # Add a force defined in the spatial frame basis
    mechanism = state.mechanism
    mech_path = path(mechanism,root_body(mechanism),bodies(mechanism)[end])
    Js = geometric_jacobian(state,mech_path)
    T_bs = relative_transform(state,root_frame(mechanism),default_frame(bodies(mechanism)[end]))
    R_bs = rotation(T_bs);
    p_bs = translation(T_bs); #-> - Rt'Ts
    hat_p_bs = [ 0 -p_bs[3] p_bs[2]; p_bs[3] 0 -p_bs[1]; -p_bs[2] p_bs[1] 0];
    # We may derive Jq.linear from Js using T_bs:
    Jq_lin = Js.linear + R_bs'* hat_p_bs*R_bs * Js.angular
    τ .+= (Jq_lin)'*[10,0,10]

end


# Define the robot:
# urdfPath = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf")
urdfPath = "pendu.urdf"
mvis, mechanism = display_urdf(urdfPath,vis)
state = MechanismState(mechanism)
set_configuration!(state, [-pi/4,pi/4])
# Indicate the end-effector
x = Point3D(default_frame(bodies(mechanism)[end]),0.0,0.0,0.0)
setelement!(mvis,x, 0.1, "p1")
# Simulate using the control! function to add an external force
qd = [pi,0]
problem = ODEProblem(Dynamics(mechanism,control!), state, (0.,10.))
sol = solve(problem, Vern7())
setanimation!(mvis, sol; realtime_rate = 1.0);

# Wrench code taken from:
# From: https://github.com/JuliaRobotics/RigidBodyDynamics.jl/issues/589
# T = Float64
# frame_foot = default_frame(findbody(mechanism, "lower_link"))
# force_1 = transform(state, FreeVector3D(root_frame(mechanism), [0.0, 0.0,  20.0]), frame_foot)
# wrench_1 = Wrench(Point3D(frame_foot, [ 0.1,  0.05, 0.0]), force_1)
#
# externalwrenches = Dict{BodyID,Wrench{T}}(
#     BodyID(findbody(mechanism, "lower_link")) => transform(state, wrench_1, root_frame(mechanism)),
# )

# result = DynamicsResult(mechanism);
# torques = [0.0,0.0]
# dynamics!(result, state, torques, externalwrenches)
