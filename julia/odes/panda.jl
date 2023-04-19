function control!(τ, t, state)
    # Set-point Regulation using PD
    # NOTE: If you had a trajectory qdes(t), you might get the desired values at t:
    # qdes, qdes_dot, qdes_ddot = traj(t)
    # Do some PD
    τ .= -diagm([20,20, 20, 20, 20, 20, 20,20,20]) * velocity(state) - diagm([100,100, 100, 100, 100, 100, 100,100,100])*(configuration(state) - [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01])
    act_sat = 50; # Actuator limits
    τ .= map( x -> x > act_sat ? act_sat : x,τ)
    τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

function plot_sol(plot_array,sol,colorarg,saveflag,savename,lab)
    qsol = vcat(sol[:]'...)
    for i=1:7
        plot!(plot_array[i],sol.t,qsol[:,i],color=colorarg,label=lab,xlabel="time",ylabel="q(t)")
    end
    pall = plot(plot_array...)
    if saveflag
        pall |> PDF(savename)
    end
    for i in 1:7
        println("final joint angle $i : $(sol[end][i])")
    end
    println("final error norm: ",norm(sol[end][1:9]-qd))
end

# Load controllers
include("panda_soln_temp.jl")
# Refresh visualization
delete!(vis)
# Load mechanism info
urdfPath = "panda.urdf"
mvis, mechanism = display_urdf(urdfPath,vis)
# Create state and set initial config and velocity
state = MechanismState(mechanism)
q0 = [0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1];
qd=[0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01]
set_configuration!(state,q0)
zero_velocity!(state)
# Update mechanism visual
set_configuration!(mvis, configuration(state))

# Create a plot array
plot_array = [] 
for i in 1:9
    push!(plot_array, plot())
end
plot(plot_array...)

# Plot desired trajectory
t_chosen =10*rand(1)[1]; println("t: ", t_chosen)
qdes, qdes_dot, qdes_ddot = traj(t_chosen)
M = hcat(qdes,qdes_dot,qdes_ddot)'
for i = 1:size(M,1)
    println(M[i,:])
end

t_range = range(0,t_chosen,100);
for i=1:9
    plot!(plot_array[i],t_range,map(x -> traj(x)[1][i],t_range),color=:black,label=:none)
end
########## Set-point reg (class example)
println("PD:")
# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol_setreg = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Plot joint angles vs time using Gadfly, indicate final error
plot_sol(plot_array,sol_setreg,[colorant"blue"],false,nothing,"PDamp")

########## Higher gains (gets < 0.01 norm error)
println("PD with higher gains:")
# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control_PD!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol_setreg_higher = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Plot joint angles vs time using Gadfly
plot_sol(plot_array,sol_setreg_higher,[colorant"gold"],false,nothing,"High PDamp")

########## Use some trajectory info
println("PD with qdot:")
# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control_PD_TT!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol_pd_tt = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Plot joint angles vs time using Gadfly
plot_sol(plot_array,sol_pd_tt,[colorant"green"],false,nothing,"PD")


########## Use some trajectory info
println("Higher gain PD with qdot:")
# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control_HPD_TT!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol_pd_tt = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Plot joint angles vs time using Gadfly
plot_sol(plot_array,sol_pd_tt,[colorant"purple"],false,nothing,"High PD")



########## Computed Torque Control
println("Computed Torque Control:")
# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control_CTC!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol_ctc = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Plot joint angles vs time using Gadfly
plot_sol(plot_array,sol_ctc,[colorant"red"],false,nothing,"CTC")

plot(plot_array...,layout=(3,3))
# Open plot in browser:
# p

# Animate solution
# setanimation!(mvis, sol; realtime_rate = 1.0);
