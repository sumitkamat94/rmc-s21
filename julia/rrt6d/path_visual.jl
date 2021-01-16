# Note: Some code adapted from TrajectoryOptimization.jl
import Pkg;
Pkg.activate(@__DIR__);
using LinearAlgebra, Plots, Random
using MeshCat, GeometryTypes, CoordinateTransformations, FileIO, MeshIO

# a number of functions:
function sampleh() # random homogenous matrix
  randvec=rand(1,3)*2*pi
  rotmat = [1 0.0 0.0; 0.0 cos(randvec[1]) -sin(randvec[1]);0.0 sin(randvec[1]) cos(randvec[1])] *[cos(randvec[1]) 0.0 -sin(randvec[1]); 0.0 1.0 0.0;sin(randvec[1]) 0.0 cos(randvec[1])]*[1 0.0 0.0; 0.0 cos(randvec[2]) -sin(randvec[2]);0.0 sin(randvec[2]) cos(randvec[2])]
  h = zeros(Float64,4,4)
  h[1:3,1:3] = rotmat;
  h[1:3,4] = randn(3,1)*100
  return h
end

function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end

function plot_sphere(vis,c1,radius,mat,name="")
    geom = HyperSphere(Point3f0(c1), convert(Float32,radius))
    setobject!(vis["sph"][name],geom,mat)
end

function add_obs!(vis,obs) # visualize obstacles
    for i in 1:size(obs,1)

        if obs[i,5] == 0.0

          plot_sphere(vis,[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"sph$i")
        else
          plot_cylinder(vis,[obs[i,1],obs[i,2],0],[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"cyl_$i")
        end
    end
end

function draw_robot(vis,h,name="")  # visualize frame as quadrotor
    # you may like to change this to an ellipsoid
    obj = "quadrotor_base.obj"

    quad_scaling = 0.3
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling

    #robot_obj = HyperRectangle(Vec(0.0,0.0,0.0),Vec(1.5,1.0,0.5))
    setobject!(vis["robot"][name],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)));
    settransform!(vis["robot"][name], compose(Translation(h[1:3,4]),LinearMap(h[1:3,1:3])))


end

#Define the obstacles
obs = [1.0 2.0 3.0 0.5 1.0; -1.0 3.0 4.0 0.5 0.0;3.0 3.0 3.0 1.0 0.0]

#Define start and goal
s=zeros(Float64,4,4);
s[1,1]=1.0;s[2,2]=1.0;s[3,3]=1.0;s[4,4]=1.0;
s[3,4]=1.0
g=zeros(Float64,4,4);
g[1,1]=1.0;g[2,2]=1.0;g[3,3]=1.0;g[4,4]=1.0;
g[1:3,4] = [5.0,6.0,4.0]

# Once you solve RRT-star, get a path using:
# traj_sol = RRT-Star(s,g,obs)

# Until then, manually create a path:
# Define trajectory as an array
traj = Array{Float64,2}[]
push!(traj,s)
for i in 1:5
     h = sampleh()
     h[1:3,4]=[i+1,0.0,1.0]
     push!(traj,h)
end
push!(traj,g)

# Convert array into tuple
traj_sol = Tuple(traj)

# Visualize everything: obs + solution path
vis = Visualizer()
open(vis)
# obstacles:
add_obs!(vis,obs)
# visualize robot at each coordinate frame on path:
for i in 1:length(traj_sol)
  draw_robot(vis,traj_sol[i],"$i")
end
