function plot_cylinder(vis,c1,c2,radius,mat,name="")
    geom = Cylinder(Point3f0(c1),Point3f0(c2),convert(Float32,radius))
    setobject!(vis["cyl"][name],geom,MeshPhongMaterial(color=RGBA(1, 0, 0, 1.0)))
end



function addcylinders!(vis,cylinders,height=1.5)
    for (i,cyl) in enumerate(cylinders)
        plot_cylinder(vis,[cyl[1],cyl[2],0],[cyl[1],cyl[2],height],cyl[3],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"cyl_$i")
    end
end

function plot_sphere(vis,c1,radius,mat,name="")
    geom = HyperSphere(Point3f0(c1), convert(Float32,radius))
    setobject!(vis["sph"][name],geom,mat)
end

function addspheres!(vis,spheres,height=1.5)
    for (i,cyl) in enumerate(spheres)
        plot_sphere(vis,[cyl[1],cyl[2],cyl[3]],2.0,MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"sph$i")
    end
end

function draw_quadrotor(vis,h,name="")
    obj = "quadrotor_base.obj"

    quad_scaling = 0.3
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling

    setobject!(vis["robot"][name],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)));

    settransform!(vis["robot"][name], compose(Translation(h[1:3,4]),LinearMap(h[1:3,1:3])))
end

function draw_quadrotor_maze(maze_cylinders)
    vis = Visualizer()
    open(vis)

    obj = "quadrotor_base.obj"

    quad_scaling = 0.3
    robot_obj = FileIO.load(obj)
    robot_obj.vertices .= robot_obj.vertices .* quad_scaling

    sphere_small = HyperSphere(Point3f0(0), convert(Float32,0.25)) # trajectory points
    sphere_med = HyperSphere(Point3f0(0), convert(Float32,0.5));
    sphere_quad = HyperSphere(Point3f0(0), convert(Float32,2.0));


    obstacles = vis["obs"]
    traj = vis["traj"]
    robot = vis["robot"]
    setobject!(vis["robot"]["quad"],robot_obj,MeshPhongMaterial(color=RGBA(0, 0, 0, 1.0)));
    # setobject!(vis["robot"]["ball"],sphere_medium,MeshPhongMaterial(color=RGBA(0, 0, 0, 0.5)));

    #settransform!(vis["/Cameras/default"], compose(Translation(0., 72., 60.),LinearMap(RotX(pi/7.5)*RotZ(pi/2))))
    addcylinders!(vis,maze_cylinders,16.0)
    addspheres!(vis,maze_cylinders,20.0)
    traj = vis["traj"]
    return vis
end
