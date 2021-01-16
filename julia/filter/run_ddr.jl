# This code is either an earlier or later version
# of slam_rotating_point.jl
function moveRobot(xTrue,v,w,Rw)
    # println(xTrue,"\nv: ",v,"\nw: ",w)
    theta = xTrue[3]
    xTrue = xTrue + [v*cos(theta),v*sin(theta),w] + Rw*randn(3,1)
    return xTrue
end

function senseWorld(xTrue,lmarks,Rw,rmax)
  R = [cos(xTrue[3]) -sin(xTrue[3]);sin(xTrue[3]) cos(xTrue[3])]
  println(R)
  d = xTrue[1:2];
  p = transpose(R)*(transpose(lmarks)-d*ones(1,size(lmarks,1)))
  q = Array{Float64,2}[]
  for i in 1:size(lmarks,1)
      x = p[1,i];y=p[2,i]
      r=sqrt(x*x+y*y)
      phi = atan(y,x)
      if r < rmax
      push!(q,[r phi])
      end
  end
  return vcat(q...)
end

# Start
Rv = diagm([0.5,0.5]) # range, bearing
Rw = diagm([0.1,0.2,0.3]) # x, y , theta
lmarks = [1.0 2.0; 2.0 4.0; -1.0 3.0]
plot([1.0],[1.0])
scatter!(lmarks[:,1],lmarks[:,2],color = :red,legend = :none)

# initialize robot
global mu = [0.0 ; 0.0;0.0];
global Sigma = diagm([0.3;0.3;0.1]);
global xTrue = [0.0;0.0;0.0]


sol = zeros(300,4)
for i in 1:300
  global mu
  global Sigma
  global xTrue
  uk = -mu;
  v = rand(1)[1]*0.1;
  w = randn(1)[1]*0.3;
  xTrue = moveRobot(xTrue,v,w,Rw);
  yk = senseWorld(xTrue,lmarks,Rw,5);
  mysys = linearizeModel(xTrue,v,w,yk,lmarks);
  mu,Sigma = filter_update(mysys,mu,Sigma,uk,yk);
  add_plot_ellipse(mu[1],mu[2],Sigma,1)
end
