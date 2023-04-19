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

function move_robot(mvis,state,mechanism,cfun)
    # state=MechanismState(mechanism)
    # update_state(state,mvis,0.0,0.0,0.0)
    problem = ODEProblem(Dynamics(mechanism,cfun), state, (0., 10.))
    sol = solve(problem, Vern7())
    # setanimation!(mvis, sol; realtime_rate = 1.0);
    for i in 1:3
        println("final joint angle $i : $(sol[end][i])")
    end
    println("final error norm: ",norm(sol[end][1:3]-qd))
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


include("wam3_soln_temp.jl")
## to restart visualization:
delete!(vis)
q0 = [-1.0,pi/2,0]
qd = [1.0;-1.0;2.0]
mvis, mechanism = display_urdf("three_link_assn2.urdf",vis)
state = MechanismState(mechanism)
set_configuration!(state,q0)
# set_configuration!(mvis, configuration(state))
# set_velocity!(state,[1.0,1.0,1.0])
# qdd = getQdesSV(state,0.0,0.0,0.0)
# inverse_dynamics(state,qdd)
println("PD:")
sol = move_robot(mvis,state,mechanism,control!);
# p = plot()
plot_sol(p,sol,[colorant"gold"],false,"foo.pdf")

println("PD higher gains:")
sol_pd_higher = move_robot(mvis,state,mechanism,control_PD!);
plot_sol(p,sol_pd_higher,[colorant"blue"],false,"foo.pdf")

println("PD with qd_dot:")
sol_pd_tt= move_robot(mvis,state,mechanism,control_PD_TT!);
plot_sol(p,sol_pd_tt,[colorant"green"],false,"foo.pdf")

println("Computed Torque Control:")
sol_ctc= move_robot(mvis,state,mechanism,control_CTC!);
plot_sol(p,sol_ctc,[colorant"gold"],false,"foo.pdf")
