import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate();
using RigidBodyDynamics, RigidBodySim, DifferentialEquations
using LinearAlgebra
using StaticArrays#, Plots
using MeshCat, MeshCatMechanisms
vis = Visualizer();open(vis)
#import PandaRobot # for visualizing Panda

function display_urdf(urdfPath,vis)
    # Displays mechanism at config all zeros
    # urdfPath must be a string
    #urdfPath = "planar3R.urdf"

    mechanism = parse_urdf(Float64,urdfPath)

    state = MechanismState(mechanism)
    zero_configuration!(state);
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdfPath),vis)
    manipulate!(state) do x
        set_configuration!(mvis, configuration(x))
    end
    return mechanism
end

# Example using Panda robot:
#urdfPath = PandaRobot.urdfpath()
#pandamech = display_urdf(urdfPath,vis)
# display_urdf("anymal.urdf",vis)
display_urdf("Cheetah.urdf",vis)
display_urdf("planar3R.urdf",vis)
