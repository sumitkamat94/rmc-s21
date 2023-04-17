urdf = joinpath(dirname(pathof(RigidBodySim)), "..", "test", "urdf", "Acrobot.urdf")
mechanism = parse_urdf(Float64, urdf)
state = MechanismState(mechanism)
set_configuration!(state, [0.1; 0.2])

function control!(τ, t, state)
    # Do some PD
    τ .= -5 .* velocity(state) - 20*(configuration(state) - [pi;0.0])
    ## keep the .
end

problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.))
sol = solve(problem, Vern7())
## Display the simulated simulate
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf),vis)
setanimation!(mvis, sol; realtime_rate = 1.0);
