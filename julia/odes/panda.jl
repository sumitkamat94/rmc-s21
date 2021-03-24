

function control!(τ, t, state)
    # Do some PD
    τ .= -diagm([20,20, 20, 20, 20, 20, 20,20,20]) * velocity(state) - diagm([100,100, 100, 100, 100, 100, 100,100,100])*(configuration(state) - [0.0;0.0;0.0;0.0;0.0;pi;0.01;0.01;0.01])
    # act_sat = 50; # Actuator limits
    # τ .= map( x -> x > act_sat ? act_sat : x,τ)
    # τ .= map( x -> x < -act_sat ? -act_sat : x,τ)
end

function plot_sol(p,sol,colorarg,saveflag,savename)
    qsol = vcat(sol[:]'...)
    for i=1:7
        push!(p,layer(x=sol.t,y=qsol[:,i],Geom.line,color=colorarg))
    end
    p
    if saveflag
        p |> PDF(savename)
    end
end

# Refresh visualization
delete!(vis)
# Load mechanism info
urdfPath = "panda.urdf"
mvis, mechanism = display_urdf(urdfPath,vis)
# Create state and set initial config and velocity
state = MechanismState(mechanism)
set_configuration!(state,[0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1])
zero_velocity!(state)
# Update mechanism visual
set_configuration!(mvis, configuration(state))

# Define ODE Problem, which defines closed loop using  control!
problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.));
# Solve ODE problem using Tsit5 scheme, and given numerical tolerances
sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
# Animate solution
setanimation!(mvis, sol; realtime_rate = 1.0);
# Plot joint angles vs time using Gadfly
plot_sol(p,sol,[colorant"red"],false,nothing)
for i in 1:7
    println("final joint angle $i : $(sol[end][i])")
end
# Show plot in browser
p
