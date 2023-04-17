

# Define environment:
x_max = 1000;
y_max = 1000;
# Green: X Y  W  H      Purple: X   Y  W  H 
obs=([300,0,300,400.0],[300 600 400 200])

# Define RRT Parameters:
EPS = 50; # the extend distance
numNodes = 1000; #4000
goalDist = 30*sqrt(2);
# Define problem instance: start, goal points
q_start = PointHP([0,0],0.0,0);
q_goal = PointHP([748,199],0.0,-1);

#Plot the problem instance
tempscatterx = [q_start.coord[1]];
tempscattery = [q_start.coord[2]];
scatter(tempscatterx,tempscattery,color = :blue,legend = :none,xlims = (-10, 1000),ylims = (-10, 1000))
tempscatterx = [q_goal.coord[1]];
tempscattery = [q_goal.coord[2]];
scatter!(tempscatterx,tempscattery,color = :red,overwrite_figure =:false)
rectangle(w, h, x, y) = Shape(x .+ [0,w,w,0], y .+ [0,0,h,h])
for i in 1:length(obs)
  plot!(rectangle(obs[i][3],obs[i][4],obs[i][1],obs[i][2]), opacity=.5)
  #print(i)
end
savefig("problem_setup.png")
#gr()
#plot(q_start.coord[1], q_start.coord[2], seriestype = :scatter, title = "My Scatter Plot")

# Begin RRT
nodes = PointHP[];
push!(nodes,q_start)

## To reduce memory, pre-allocate variables:
normstore=[0.0];
point1 = zeros(2);
point2 = zeros(2);
q_rand = zeros(2);
treestore = zeros(numNodes,10)
# Setup the animation recording

anim = Plots.Animation()
# Plots.frame(anim)

solveRRT(nodes,anim,q_rand,normstore,treestore)
D = Float64[];
for j in 1:length(nodes)
    tmpdist = rrthp.dist(nodes[j].coord, q_goal.coord);
    push!(D,tmpdist);
end

# Search backwards from goal to start to find the optimal least cost path
val = minimum(D);
idx=argmin(D)
q_final = nodes[idx];
q_goal = PointHP(q_goal.coord,q_goal.cost,idx)
push!(nodes,q_goal)
q_end = PointHP(q_goal.coord,q_goal.cost,idx);
opt_path=Int64[]
push!(opt_path,idx)
global start =idx;
while start!= 0
    global start
    tempindex = start
    global start = nodes[start].parent
    #println("current node:",tempindex,", parent node: ",start)
    if start!=0
    plot!([nodes[start].coord[1], nodes[tempindex].coord[1]], [nodes[start].coord[2], nodes[tempindex].coord[2]],color = :red);
    # Plots.frame(anim)
    end
    push!(opt_path,start)
end
for kk in nodes
    scatter!([kk.coord[1]],[kk.coord[2]],markershape = :diamond,markersize = 3,markercolor = :red)
    if kk.parent > 0
    plot!([kk.coord[1],nodes[kk.parent].coord[1]],[kk.coord[2],nodes[kk.parent].coord[2]],markershape = :diamond,markersize = 3,markercolor = :red)
    end
end
savefig("rrt_solution.png")
println("optimal path in reverse: ",opt_path)
# gif(anim, "rrt_anim6_fps10.gif", fps = 60)

# Final result appears to be in the plot, not a data structure


