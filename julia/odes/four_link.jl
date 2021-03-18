# assumes that mvis and mechanism have been created using display_urdf

function update_state(state,mvis,q1,q2,q3,q4)
    set_configuration!(state, [q1, q2,q3,q4])
    set_configuration!(mvis, configuration(state))
end

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;-1.0;2.0;-1.0])
end

function move_robot(mvis,mechanism)
    state=MechanismState(mechanism)
    update_state(state,mvis,0.0,0.0,0.0,0.0)
    problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 2.))
    sol = solve(problem, Vern7())
    setanimation!(mvis, sol; realtime_rate = 1.0);
end

## to restart visualization:
delete!(vis)
mvis, mechanism = display_urdf("four_link_WAM.urdf",vis)

move_robot(mvis,mechanism)
