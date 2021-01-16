r_quad_maze = 2.0
r_cylinder_maze = 2.0
maze_cylinders = []
zh = 3
l1 = 5
l2 = 4
l3 = 4
l4 = 10

for i = range(-25,stop=-10,length=l1)
    push!(maze_cylinders,(i, 10,r_cylinder_maze))
end

for i = range(10,stop=25,length=l1)
    push!(maze_cylinders,(i, 10, r_cylinder_maze))
end

for i = range(-5,stop=5,length=l3)
    push!(maze_cylinders,(i, 30, r_cylinder_maze))
end

for i = range(-25,stop=-10,length=l1)
    push!(maze_cylinders,(i, 50, r_cylinder_maze))
end

for i = range(10,stop=25,length=l1)
    push!(maze_cylinders,(i, 50, r_cylinder_maze))
end

for i = range(10+2*r_cylinder_maze,stop=50-2*r_cylinder_maze,length=l4)
    push!(maze_cylinders,(-25, i, r_cylinder_maze))
end

for i = range(10+2*r_cylinder_maze,stop=50-2*r_cylinder_maze,length=l4)
    push!(maze_cylinders,(25, i, r_cylinder_maze))
end


rrt6d.draw_quadrotor_maze(maze_cylinders)
