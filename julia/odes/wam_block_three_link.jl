# assumes that mvis and mechanism have been created using display_urdf

function update_state(state,mvis,q1,q2,q3)
    set_configuration!(state, [q1, q2,q3])
    set_configuration!(mvis, configuration(state))
end

function control!(τ, t, state)
    # Do some PD
    τ.=0
    # Need to set to zeros otherwise values are undefined in-place one from earlier,
    # Causing weird motion
    # Since the bloc joint is first, torques 1:6 are bloock, 7:9 is 3link torques
    # Config 1:7 is block quarternion, 8:10 is 3link joint angles
    τ[7:9] .= -20 .* velocity(state)[7:9] - 100*(configuration(state)[8:10] - qd)
    # detect contact by placing tip of robot in block frame
    block_b = findbody(mechanism, "block")
    upper_link_b = findbody(mechanism, "upper_link")
    tip_point = Point3D(default_frame(upper_link_b),0.0,0.0,1.0)
    UpperToBlock = relative_transform(state,default_frame(upper_link_b),default_frame(block_b))
    tip_in_block = UpperToBlock*tip_point;

    if (abs(tip_in_block.v[1]) <= 1.00) && (abs(tip_in_block.v[2])<=1.0 )  && (abs(tip_in_block.v[3])<=1.0 )
        # Contact model
        F_scalar = 50e3*(1 - tip_in_block.v[1]);
        # Since we're in the body frame, and force is
        # aligned along negative x, the wrench on this frame is
        # easy: [r x f; f]
        # τ[1:3] .= LinearAlgebra.cross(tip_in_block.v,[-F_scalar,0.0,0.0])
        # τ[4:6] .= [-F_scalar,0.0,0.0]
        # Map this force in block to opposite force acting in body frame of...
        # Robot end-effector
        rot = rotation(UpperToBlock)
        F_in_robot = rot'*[F_scalar,0.0,0.0];
        mech_path = path(mechanism,bodies(mechanism)[1],upper_link_b)
        Jb = point_jacobian(state,mech_path,tip_point)
        tau_F = (Jb.J)'*F_in_robot
        τ[7:9] .+= tau_F[7:9]
    end
    # τ[1:6].+=-1.0*velocity(state)[1:6]
    # println("t:,",t,", flag:",tempflag, ", F:", F_scalar)
    # println("tau:",τ)

end

function move_robot(mvis,state,mechanism,cfun)
    # state=MechanismState(mechanism)
    # update_state(state,mvis,0-1.0,pi/2,0)
    problem = ODEProblem(Dynamics(mechanism,cfun), state, (0., 10.0))
    sol = solve(problem, Tsit5())
    for i in 1:3
        println("final joint angle $i : $(sol[end][i])")
    end
    println("final error norm: ",norm(sol[end][1:3]-qd))
    # setanimation!(mvis, sol; realtime_rate = 1.0);
    return sol
end

function plot_sol(p,sol,colorarg,saveflag,savename)
    qsol = vcat(sol[:]'...)
    for i=1:3
        push!(p,layer(x=sol.t,y=qsol[:,i],Geom.line,color=colorarg))
    end
    p
    if saveflag
        p |> PDF(savename)
    end
end


function getQdesSV(state,a,b,c)
    # Creates the SegmentedVector structure using the state
    qdd_des = similar(velocity(state))
    qdd_des[1] = a;
    qdd_des[2] = b;
    qdd_des[3] = c;
    return qdd_des
end


function add_contact_wall!(mechanism)


    body_platform = findbody(mechanism,"base_platform");
    # hs_p = Point3D(root_frame(mechanism),0.0,0.0,0.0)
    # hs_v = FreeVector3D(root_frame(mechanism),0.0,0.0,1.0)
    platform_frame = default_frame(body_platform)
    world_frame = root_frame(mechanism)
    ce = RigidBodyDynamics.Contact.ContactEnvironment{Float64}()
    # Ground:
    hs_p = Point3D(world_frame,0.0,0.0,0.0)
    hs_v = FreeVector3D(world_frame,0.0,0.0,1.0)
    hs = RigidBodyDynamics.Contact.HalfSpace3D(hs_p,hs_v)
    push!(ce.halfspaces,hs)
    mechanism.environment = ce
    # Define contact models
    conmod1 = HuntCrossleyModel(50e3,1.5*0.2*50e3,1.5)
    conmod2 = ViscoelasticCoulombModel(0.1,10e5,10e3)
    conmod3 = SoftContactModel(conmod1,conmod2)

    # conmod3 = SoftContactModel(0,0)
    # upperlink_body = findbody(mechanism, "upper_link")
    # cpb = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(upperlink_body),0.0,0.0,1.0),conmod3)
    # add_contact_point!(upperlink_body,cpb)
    block_body = findbody(mechanism, "block")
    cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),1.0,1.0,1.0),conmod3)
    add_contact_point!(block_body,cp1)
    cp2 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),-1.0,-1.0,-1.0),conmod3)
    add_contact_point!(block_body,cp2)
    cp3 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),1.0,1.0,-1.0),conmod3)
    add_contact_point!(block_body,cp3)
    cp4 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),1.0,-1.0,-1.0),conmod3)
    add_contact_point!(block_body,cp4)
    cp5 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),1.0,-1.0,1.0),conmod3)
    add_contact_point!(block_body,cp5)
    cp6 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),-1.0,1.0,-1.0),conmod3)
    add_contact_point!(block_body,cp6)
    cp7 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),-1.0,-1.0,1.0),conmod3)
    add_contact_point!(block_body,cp7)
    cp8 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(block_body),-1.0,1.0,1.0),conmod3)
    add_contact_point!(block_body,cp8)
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


## to restart visualization:
delete!(vis)
# p=plot()
# q0 = [-1.0,pi/2,0]
# qd = [1.0;-1.0;2.0]
qd = [0.0;1.0;1.0]

mechanism=parse_urdf("three_link_block.urdf")
remove_fixed_tree_joints!(mechanism)

# Include contacts:
import RigidBodyDynamics.Contact.ViscoelasticCoulombModel
import RigidBodyDynamics.Contact.HuntCrossleyModel
import RigidBodyDynamics.Contact.SoftContactModel
add_contact_wall!(mechanism)


state = MechanismState(mechanism)
rand_configuration!(state)
zero_velocity!(state)
# qdd = getQdesSV(state,2.0,2.0,2.0)
# inverse_dynamics(state,qdd)
#
mvis = MechanismVisualizer(mechanism, URDFVisuals("three_link_block.urdf"),vis)
set_configuration!(state, findjoint(mechanism, "elbow"), 2.0)
set_configuration!(state, findjoint(mechanism, "shoulder"), -1)
set_configuration!(state, findjoint(mechanism, "trunk"), 0.0)
set_configuration!(state, findjoint(mechanism, "block_float_joint"), [0.0,0.0,0.0,1.0,2.0,0.5,1.0])
set_configuration!(mvis, configuration(state))


sol = move_robot(mvis,state,mechanism,control!)
setanimation!(mvis, sol; realtime_rate = 1.0);
