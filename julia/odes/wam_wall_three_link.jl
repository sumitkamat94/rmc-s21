# assumes that mvis and mechanism have been created using display_urdf

function update_state(state,mvis,q1,q2,q3)
    set_configuration!(state, [q1, q2,q3])
    set_configuration!(mvis, configuration(state))
end

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;-1.0;2.0])
    τ .= map( x -> x > 50 ? 50 : x,τ)
    τ .= map( x -> x < -50 ? -50 : x,τ)
end

function control_PDplus!(τ, t, state)
    # Do some PD:

    # Get G(q):
    qdd_zero = similar(velocity(state)) # Acceleration is a specific datatype
    for i=1:3
        qdd_zero[i]=0
    end
    qd= deepcopy(state)
    zero_velocity!(qd)
    G_from_ID = inverse_dynamics(qd, qdd_zero);

    τ .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;-1.0;2.0])+G_from_ID
    τ .= map( x -> x > 50 ? 50 : x,τ)
    τ .= map( x -> x < -50 ? -50 : x,τ)
end

function move_robot(mvis,state,mechanism,cfun)
    # state=MechanismState(mechanism)
    # update_state(state,mvis,0-1.0,pi/2,0)
    problem = ODEProblem(Dynamics(mechanism,cfun), state, (0., 10.))
    sol = solve(problem, Vern7())
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
    # Wall:
    hs_p = Point3D(world_frame,0.0,0.0,0.0)
    hs_v = FreeVector3D(world_frame,0.0,-1.0,0.0)
    hs = RigidBodyDynamics.Contact.HalfSpace3D(hs_p,hs_v)
    ce = RigidBodyDynamics.Contact.ContactEnvironment{Float64}()
    push!(ce.halfspaces,hs)
    # Ground:
    hs_p = Point3D(world_frame,0.0,0.0,0.0)
    hs_v = FreeVector3D(world_frame,0.0,0.0,1.0)
    hs = RigidBodyDynamics.Contact.HalfSpace3D(hs_p,hs_v)
    push!(ce.halfspaces,hs)
    mechanism.environment = ce


    conmod1 = HuntCrossleyModel(50e3,1.5*0.2*50e3,1.5)
    conmod2 = ViscoelasticCoulombModel(0.1,10e5,10e3)
    conmod3 = SoftContactModel(conmod1,conmod2)
    #conmod3 = SoftContactModel(0,0)
    cpb = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[4]),0.0,0.0,1.0),conmod3)
    add_contact_point!(bodies(mechanism)[4],cpb)
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


include("wam3_soln_temp.jl")
## to restart visualization:
delete!(vis)
p=plot()
q0 = [-1.0,pi/2,0]
qd = [1.0;-1.0;2.0]


mechanism=parse_urdf("three_link_wall.urdf")
remove_fixed_tree_joints!(mechanism)

# Include contacts:
import RigidBodyDynamics.Contact.ViscoelasticCoulombModel
import RigidBodyDynamics.Contact.HuntCrossleyModel
import RigidBodyDynamics.Contact.SoftContactModel
add_contact_wall!(mechanism)


state = MechanismState(mechanism)
set_configuration!(state,q0)


zero_velocity!(state)
# qdd = getQdesSV(state,2.0,2.0,2.0)
# inverse_dynamics(state,qdd)
#
mvis = MechanismVisualizer(mechanism, URDFVisuals("three_link_wall.urdf"),vis)
set_configuration!(mvis, configuration(state))

println("PD:")
sol_pd = move_robot(mvis,state,mechanism,control!);
plot_sol(p,sol_pd,[colorant"red"],false,"foo.pdf")
println("PD + (GC):")
sol_pdplus = move_robot(mvis,state,mechanism,control_PDplus!);
plot_sol(p,sol_pdplus,[colorant"red"],false,"foo.pdf")


set_configuration!(state,sol_pdplus.u[end][1:3])

# Create base and end-effector frames:
ee_frame=default_frame(bodies(mechanism)[end])
base_frame = default_frame(bodies(mechanism)[1])
# Define point in frame of end effector
x = Point3D(ee_frame, 1.0, 0.0, 0.0)
# Visualize this point
setelement!(mvis,x, 0.05, "p1") # mvis, point, radius, name

set_configuration!(state,sol_pd.u[end][1:3])
calcXinW(state,x)
set_configuration!(state,sol_pdplus.u[end][1:3])
calcXinW(state,x)

# p
