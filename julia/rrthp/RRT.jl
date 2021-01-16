# Node/Point data structure
struct PointHP
  coord::Array{Float64,2}
  cost::Float64
  parent::Int
end

# Define environment:
x_max = 1000;
y_max = 1000;
obs=([300,0,300,400.0],[300 450 400 200])

# Define RRT Parameters:
EPS = 20;
numNodes = 10000; #4000

# Define problem instance: start, goal points
q_start = PointHP([0 0],0.0,0);
q_goal = PointHP([748 199],0.0,-1);

#Plot the problem instance
tempscatterx = [q_start.coord[1]];
tempscattery = [q_start.coord[2]];
scatter(tempscatterx,tempscattery,color = :blue,legend = :none)
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

for i in 1:numNodes
    # Break if goal node is already reached by existing node
    endflag = false;
    for j = 1:length(nodes)
        #print(norm(nodes[j].coord - q_goal.coord))
        if norm(nodes[j].coord - q_goal.coord)<= 25*sqrt(2)
            endflag = true;
            break
        end
    end
    if endflag
        break
    end

    # Sample and plot
    q_rand = [rand(1)*x_max rand(1)*y_max];
    tempscatterx = [q_rand[1]];
    tempscattery = [q_rand[2]];
    scatter!(tempscatterx,tempscattery,markershape = :x)

    # Pick the closest node from existing list to branch out from
    ndist = Float64[];
    for j in 1:length(nodes)
        n = nodes[j];
        tmp = rrthp.dist(n.coord, q_rand);
        push!(ndist,tmp);
    end
    val = minimum(ndist);
    idx=argmin(ndist)
    q_near = nodes[idx];

    # Steer nearest node towards random sample
    temp = rrthp.steer(q_rand, q_near.coord, val, EPS)
    q_new = PointHP([temp[1] temp[2]],0.0,-1);
    coll_flag = rrthp.noCollision(q_new.coord, q_near.coord, obs)
    #println(coll_flag)
    if coll_flag
        plot!([q_near.coord[1], q_new.coord[1]], [q_near.coord[2], q_new.coord[2]]);
        temp = q_new.coord;
        tempcost = rrthp.dist(q_new.coord, q_near.coord) + q_near.cost;
        q_new = PointHP([temp[1] temp[2]],tempcost,-1);
        # Within a radius of r, find all existing nodes
        # we use the parent field of PointHP to store which element of node
        # has been added to q_nearest
        q_nearest = PointHP[];
        r = 60;
        for j in 1:length(nodes)
            if rrthp.noCollision(nodes[j].coord, q_new.coord, obs) && rrthp.dist(nodes[j].coord, q_new.coord) <= r
                push!(q_nearest, PointHP(nodes[j].coord,nodes[j].cost,j))
            end
        end
        # Initialize cost to currently known value
        q_min = q_near;
        C_min = q_new.cost;

        # Iterate through all nearest neighbors to find alternate lower
        # cost paths

        for k in 1:length(q_nearest)
            if rrthp.noCollision(q_nearest[k].coord, q_new.coord, obs) && q_nearest[k].cost + rrthp.dist(q_nearest[k].coord, q_new.coord) < C_min
                q_min = q_nearest[k];
                C_min = q_nearest[k].cost + rrthp.dist(q_nearest[k].coord, q_new.coord);

            end
        end

        # Update parent to least cost-from node
        for j in 1:length(nodes)
            if nodes[j].coord == q_min.coord
                q_new = PointHP(q_new.coord,q_new.cost,j);
            end
        end

        # Append to nodes
        push!(nodes,q_new);
    end
end

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
    end
    push!(opt_path,start)
end
savefig("rrt_solution.png")
println("optimal path in reverse: ",opt_path)
# Final result appears to be in the plot, not a data structure
