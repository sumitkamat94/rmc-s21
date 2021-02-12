# instructions from:
# https://github.com/JuliaRobotics/MeshCatMechanisms.jl/blob/master/examples/interactive_manipulation.ipynb
# assumes subfolders:
# currentfolder
#   |-PandaRobot
#       |-deps
#           |-Panda
#               |-meshes
#               |-panda.urdf
# Assumes: you have added the PandaRobot package to the current environment
# and in Julia REPL, import PandaRobot

# only displays Panda in a configuration
# Docs on RigidBodyDynamics provide more functions for mechanism and state
#import PandaRobot # This import should be run in startup.jl

function control!(τ, t, state)
    # Do some PD
    τ .= -20 .* velocity(state) - 100*(configuration(state) - [0.0;0.0;0.0;-0.0;0.0;pi;0.01;0.01;0.01])
end


urdfPath = "panda.urdf"
mechanism = parse_urdf(Float64,urdfPath)

state = MechanismState(mechanism)
set_configuration!(state,[0.01;-0.5;-0.0;-2.0;-0.3;1.5;-0.7;0.1;0.1])
#zero_configuration!(state);
#set_configuration!(state,[pi/6;pi/6;pi/6]);
zero_velocity!(state)
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
#for bd in bodies(mechanism)
#    setelement!(mvis,default_frame(bd),0.5,"$bd")
#end
#setelement!(mvis,default_frame(bodies(mechanism)[12]),0.5,"$bodies(mechanism)[11]")
manipulate!(state) do x
    set_configuration!(mvis, configuration(x))
end

problem = ODEProblem(Dynamics(mechanism,control!), state, (0., 10.));
sol = solve(problem, Tsit5(),reltol=1e-8,abstol=1e-8);
mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
setanimation!(mvis, sol; realtime_rate = 1.0);
