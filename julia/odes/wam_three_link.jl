# assumes that mvis and mechanism have been created using display_urdf

function update_state(state,mvis,q1,q2,q3)
    set_configuration!(state, [q1, q2,q3])
    set_configuration!(mvis, configuration(state))
end

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [1.0;-1.0;2.0])
    τ .= map( x -> x > 10 ? 10 : x,τ)
    τ .= map( x -> x < -10 ? -10 : x,τ)
end

function move_robot(mvis,mechanism)
    state=MechanismState(mechanism)
    update_state(state,mvis,0.0,0.0,0.0)
    problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 5.))
    sol = solve(problem, Vern7())
    setanimation!(mvis, sol; realtime_rate = 1.0);

    ### so some plotting


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
## to restart visualization:
delete!(vis)
mvis, mechanism = display_urdf("three_link_assn2.urdf",vis)
state = MechanismState(mechanism)
set_configuration!(state,[0.5,0.5,0.5])
set_configuration!(mvis, configuration(state))
set_velocity!(state,[1.0,1.0,1.0])
qdd = getQdesSV(state,2.0,2.0,2.0)
inverse_dynamics(state,qdd)
sol = move_robot(mvis,mechanism);
# p = plot()
plot_sol(p,sol,[colorant"gold"],false,"foo.pdf")
println("final state: ", sol[end])
p
