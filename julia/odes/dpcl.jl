urdf = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(Float64, urdf)
state = MechanismState(mechanism)
set_configuration!(state, [0.1; 0.2])

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [pi;0.0])
end

problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.))
sol = solve(problem, Vern7())
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf),vis)
setanimation!(mvis, sol; realtime_rate = 1.0);
