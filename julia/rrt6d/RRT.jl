
# Define Environment with Obstacles
x_max = 11;
y_max = 11;
z_max = 11;
obs = zeros(Float64, 5, 5);
obs[1,:]=[4.0,4.0,4.0,2.0,1.0];
obs[2,:]=[4.0,14.0,4.0,1.0,1.0];
obs[3,:]=[4.0,4.0,14.0,2.0,1.0];
obs[4,:]=[14.0,4.0,4.0,1.0,1.0];
obs[5,:]=[4.0,20.0,20.0,1.0,1.0];

nobs=size(obs,1)

EPS = 4;
numNodes = 4000; #4000

green_mat = MeshPhongMaterial(color=RGBA(0, 1.0, 0.0, 1.0))
blue_mat = MeshPhongMaterial(color=RGBA(0, 0, 1.0, 1.0))
red_mat = MeshPhongMaterial(color=RGBA(1.0, 0, 0, 1.0))
black_mat = MeshPhongMaterial(color=RGBA(0.0, 0, 0, 1.0))
temph = zeros(Float64, 4, 4);
temph[1,1]=1.0;
temph[2,2]=1.0;
temph[3,3]=1.0;
temph[1:3,4] = [0, 0.0, 2.0];
q_start = rrt6d.FrameHP(temph,0.0,0);
temph2 = zeros(Float64, 4, 4);
temph2[1,1]=1.0;
temph2[2,2]=1.0;
temph2[3,3]=1.0;
temph2[1:3,4] = [10, 10.0, 10.0];
q_goal = rrt6d.FrameHP(temph2,0.0,0);

nodes = rrt6d.FrameHP[];
push!(nodes,q_start)

println("samples:")
for i in 1:numNodes
    # Break if goal node is already reached by existing node
    endflag = false;
    for j = 1:length(nodes)

        if rrt6d.dist(nodes[j].h,q_goal.h)<= EPS
            endflag = true;
            break
        end
    end
    if endflag
        break
    end
    if mod(i,5) == 0
      h_rand = q_goal.h
    else
      h_rand = rrt6d.sampleh()
    end

    # Pick the closest node from existing list to branch out from
    ndist = Float64[];
    for j in 1:length(nodes)
        tmp = rrt6d.dist(nodes[j].h, h_rand);
        push!(ndist,tmp);
    end
    val = minimum(ndist);
    idx=argmin(ndist)
    h_near = nodes[idx];

    # Steer nearest node towards random sample
    temp = rrt6d.steer(h_rand, h_near.h, val, EPS)
    h_new = rrt6d.FrameHP(temp,0.0,idx);
    coll_flag = rrt6d.noCollision(h_new.h, h_near.h, obs)
    #println(coll_flag)
    if coll_flag
      tempcost = rrt6d.dist(h_new.h, h_near.h) + h_near.cost;
      h_new = rrt6d.FrameHP(h_new.h,tempcost,idx);

      # find all nearest nodes
      h_nearest = rrt6d.FrameHP[];
      r = 2*EPS;
      for j in 1:length(nodes)
          if rrt6d.noCollision(nodes[j].h, h_new.h, obs) && rrt6d.dist(nodes[j].h, h_new.h) <= r
              push!(h_nearest, rrt6d.FrameHP(nodes[j].h,nodes[j].cost,j))
          end
      end
      #println("length h_nearest: ", length(h_nearest))
      # Initialize cost to currently known value
      q_min = h_new;
      C_min = h_new.cost;

      # Iterate through all nearest neighbors to find alternate lower cost paths
      for k in 1:length(h_nearest)
          if rrt6d.noCollision(h_nearest[k].h, h_new.h, obs) && h_nearest[k].cost + rrt6d.dist(h_nearest[k].h, h_new.h) < C_min
              q_min = h_nearest[k];
              C_min = h_nearest[k].cost + rrt6d.dist(h_nearest[k].h, h_new.h);
              #println("i: ",i,", k : ",k)
          end
      end

      # Update parent to least cost-from node
      for j in 1:length(nodes)
          if rrt6d.dist(nodes[j].h,q_min.h) < 0.1
              h_new = rrt6d.FrameHP(h_new.h,h_new.cost,j);
              break
          end
      end

      # Append to nodes
      push!(nodes,h_new);
    end
end

D = Float64[];
for j in 1:length(nodes)
    tmpdist = rrt6d.dist(nodes[j].h, q_goal.h);
    push!(D,tmpdist);
end

traj = Array{Float64,2}[]

# Search backwards from goal to start to find the optimal least cost path
val = minimum(D);
idx=argmin(D)
println("final node before goal:", idx)
q_goal = rrt6d.FrameHP(q_goal.h,q_goal.cost,idx)
push!(nodes,q_goal)
push!(traj,q_goal.h)
global start =idx;
while start!= 0
    global start
    tempindex = start
    global start = nodes[start].parent
    if start!=0
      push!(traj,nodes[start].h)
    end
    #println("start index: ",start,"tempindex: ",tempindex)
end


# Convert array into tuple
traj_sol = Tuple(traj);


# Visualize everything: obs + solution path
# open(vis2)
delete!(vis2)
# obstacles:
rrt6d.add_obs!(vis2,obs)
# all nodes
for i in 1:length(nodes)
  rrt6d.draw_quadrotor(vis2,nodes[i].h,black_mat,"allnode$i")
end

# visualize robot at each coordinate frame on path:
for i in 1:length(traj_sol)
  rrt6d.draw_quadrotor(vis2,traj_sol[i],green_mat,"traj$i")

  if i<length(traj_sol)
  #println("i: ",i,", length: ",length(traj_sol))
  coordinates = [Point(traj_sol[i][1,4], traj_sol[i][2,4], traj_sol[i][3,4]), Point(traj_sol[i+1][1,4], traj_sol[i+1][2,4], traj_sol[i+1][3,4])]
setobject!(vis2["line"]["line$i"], Object(PointCloud(coordinates), LineBasicMaterial(), "Line"))
  end
end
rrt6d.draw_quadrotor(vis2,q_start.h,blue_mat,"startfin")
rrt6d.draw_quadrotor(vis2,q_goal.h,red_mat,"goalfin")

global pathcost = 0.0
for i in 1:length(traj_sol)-1
  global pathcost
  pathcost = pathcost + rrt6d.cost(traj_sol[i],traj_sol[i+1])
end
println("path cost: ", pathcost)
