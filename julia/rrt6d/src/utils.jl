function sampleh()
  randvec=randn(1,3)*pi
  rotmat = [1 0.0 0.0; 0.0 cos(randvec[1]) -sin(randvec[1]);0.0 sin(randvec[1]) cos(randvec[1])] *[cos(randvec[1]) 0.0 sin(randvec[1]); 0.0 1.0 0.0;-sin(randvec[1]) 0.0 cos(randvec[1])]*[1 0.0 0.0; 0.0 cos(randvec[2]) -sin(randvec[2]);0.0 sin(randvec[2]) cos(randvec[2])]
  h = zeros(Float64,4,4)
  h[1:3,1:3] = rotmat;
  randvec = randn(2,1)
  h[1,4] = randvec[1]*20
  h[2,4] = randvec[2]*20
  randvec = rand(1)
  h[3,4]=20*randvec[1]
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

function add_obs!(vis,obs)
    for i in 1:size(obs,1)
        if obs[i,5] == 0.0
          plot_sphere(vis,[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"sph$i")
        else
          #println(obs[i,:])
          plot_cylinder(vis,[obs[i,1],obs[i,2],0],[obs[i,1],obs[i,2],obs[i,3]],obs[i,4],MeshPhongMaterial(color=RGBA(0, 0, 1, 1.0)),"cyl_$i")
        end
    end
end

function draw_quadrotor(vis,h,mat,name="")
    #obj = "quadrotor_base.obj"

    #quad_scaling = 1.5
    #robot_obj = FileIO.load(obj)
    #robot_obj.vertices .= robot_obj.vertices .* quad_scaling
    robot_obj = HyperRectangle(Vec(-0.75,-0.5,-0.25),Vec(1.5,1.0,0.5))
    setobject!(vis["robot"][name],robot_obj,mat);

    settransform!(vis["robot"][name], compose(Translation(h[1:3,4]),LinearMap(h[1:3,1:3])))
end
