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
# traj_sol = RRT-Star(traj)
#Define trajectory as an array
traj = Array{Float64,2}[]
push!(traj,s)
for i in 1:5
     h = rrt6d.sampleh()
     h[1:3,4]=[i+1,0.0,1.0]
     push!(traj,h)
end
push!(traj,g)

# Convert array into tuple
traj_sol = Tuple(traj)

#Visualize everything: obs + solution path
vis = Visualizer()
open(vis)
# obstacles:
rrt6d.add_obs!(vis,obs)
# quadrotors at each node:
for i in 1:length(traj_sol)
  rrt6d.draw_quadrotor(vis,traj_sol[i],"$i")
end
