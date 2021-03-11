# assumes that mvis and mechanism have been created using display_urdf

function update_state(state,mvis,q1,q2,q3)
    set_configuration!(state, [q1, q2,q3])
    set_configuration!(mvis, configuration(state))
end

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;-1.0;2.0])
end

function move_robot(mvis,mechanism)
    state=MechanismState(mechanism)
    update_state(state,mvis,0.0,0.0,0.0)
    problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 2.))
    sol = solve(problem, Vern7())
    setanimation!(mvis, sol; realtime_rate = 1.0);
end

function getQdesSV(state,a,b,c)
    # Creates the SegmentedVector structure using the state
    qdd_des = similar(velocity(state))
    qdd_des[1] = a;
    qdd_des[2] = b;
    qdd_des[3] = c;
    return qdd_des
end
## to restart visualization:
delete!(vis)
mvis, mechanism = display_urdf("three_link_assn2.urdf",vis)
state = MechanismState(mechanism)
set_configuration!(state,[0.5,0.5,0.5])
set_configuration!(mvis, configuration(state))
set_velocity!(state,[1.0,1.0,1.0])
qdd = getQdesSV(state,2.0,2.0,2.0)
inverse_dynamics(state,qdd)
# move_robot(mvis,mechanism)
