function linearizePointModel(xTrue,Rvm,Rwm,yk,lmarks)
    nl = size(lmarks,1)
    # Initialize matrices
    A = zeros(2+2*nl,2+2*nl)
    B = zeros(2+2*nl,2)
    C = zeros(2*nl,2+2*nl)
    Rv = zeros(2*nl,2*nl)
    Rw = zeros(2+2*nl,2+2*nl)

    # Start populating
    A[1,1] = 1.0;
    A[1,2] = 0.0;
    A[2,1] = 0.0;
    A[2,2] = 1.0;
    B[1,1] = 1.0;
    B[2,2] = 1.0;
    Rw[1:2,1:2] = Rwm
    for i in 1:nl
        A[2+2*i-1,2+2*i-1] = 1.0;
        A[2+2*i  ,2+2*i] = 1.0;
        Rv[(2*i-1):(2*i),(2*i-1):(2*i)] = Rvm; # all landmarks get noise
        # Rw[(2+2*i-1):(2+2*i),(2+2*i-1):(2+2*i)] = 0.1*Rwm;
        C[2*i-1,1] = -1.0;
        C[2*i,2] = -1.0;
        C[2*i-1,2+2*i-1] = 1.0;
        C[2*i  ,2+2*i] = 1.0;
    end
    mysys = LTIsys(A,B,C,Rw,Rv)
    return mysys
end

# Start
Rv = diagm([0.4;0.4]) # x, y measurement noise
Rw = diagm([0.06;0.04]) # x, y  process noise
lmarks = [1.0 2.0; 2.0 4.0; -1.0 3.0; -1 -2]
nl = size(lmarks,1)
gr()
plot([1.0],[1.0])
scatter!(lmarks[:,1],lmarks[:,2],color = :orange,legend = :none)

allmu=[]
# initialize robot
global mu = zeros(2+2*nl);
global Sigma = diagm(15*ones(2+2*nl));
global xTrue = [4.0;4.0]
# println("Sigma:",Sigma)
push!(allmu,[mu[1]-xTrue[1] mu[2]-xTrue[2]])
# Get model (globally linear)
mysys = linearizePointModel(xTrue,Rv,Rw,0.0,lmarks);
anim = Plots.Animation()
scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)
scatter!([mu[1]],[mu[2]],color = :red,legend = :none)
scatter!([mu[3:2:end]],[mu[4:2:end]],color = :blue,legend = :none)
add_plot_ellipse(mu[1],mu[2],Sigma[1:2,1:2],0)
Plots.frame(anim)
# This loop simulates both real world and agent's imagined world
# Integrator is Euler, unlike true ODE simulators
for i in 1:100
  global mu
  global Sigma
  global xTrue
  # Construct control, adding process noise
  v = - 0.0*xTrue[1]+0.1;
  w = - 0.0*xTrue[2]+0.1;
  uk = [v;w];
  # Update true model in response to noisy input
  xTrue = mysys.A[1:2,1:2]*xTrue + uk + 1.0*Rw*randn(2,1);
  # Get a true measurement from all landmarks
  yk = zeros(2*nl);
  for j in 1:nl
    yk[2*j-1] = lmarks[j,1] - xTrue[1] #relative x
    yk[2*j] = lmarks[j,2] - xTrue[2] # relative y
  end
  # Add measurement noise
  yk = yk+mysys.Rv*randn(2*nl,1)
  # Update the filter based on commanded control and measurement
  mu,Sigma = filter_update(mysys,mu,Sigma,uk,yk);
  # println("Sigma:",Sigma)
  scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)
  scatter!([mu[1]],[mu[2]],color = :red,legend = :none)
  scatter!([mu[3:2:end]],[mu[4:2:end]],color = :blue,legend = :none)
  push!(allmu,[mu[1]-xTrue[1] mu[2]-xTrue[2]])
  add_plot_ellipse(mu[1],mu[2],Sigma[1:2,1:2],0)
  for j in 1:nl
    add_plot_ellipse(mu[2+2*j-1],mu[2+2*j],Sigma[(2+2*j-1):(2+2*j),(2+2*j-1):(2+2*j)],1)
  end
  Plots.frame(anim)
end
gif(anim, "slam_simple_fps20.gif", fps = 20)


plot([1.0],[1.0])
# # Post-processing
# derror = -mu[3:4]+lmarks[1,:]
derror = -0.25*[1 1 1 1]*(hcat(mu[3:2:end],mu[4:2:end])-lmarks);
println("true mapping error: ",xTrue -mu[1:2])
println("average landmark error: ",derror)
# plot landmark-corrected estimates of landmark
for j in 1:nl
  add_plot_ellipse(mu[2+2*j-1]+derror[1],mu[2+2*j]+derror[2],Sigma[(2+2*j-1):(2+2*j),(2+2*j-1):(2+2*j)],1)
end
scatter!(lmarks[:,1],lmarks[:,2],color = :red,legend = :none,fillalpha=0.5)

# plot landmark-corrected estimates of robot position
scatter!([mu[1]+derror[1]],[mu[2]+derror[2]],color = :orange,legend = :none)
add_plot_ellipse(mu[1]+derror[1],mu[2]+derror[2],Sigma[1:2,1:2],0)

# plot landmark-corrected true robot position
scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)

plot!([1.0],[1.0],grid=:true)

savefig("simple_kf.png")




allmu=vcat(allmu...);
my_colors = [cgrad(:bluesreds, [0.01, 0.99])[z] for z ∈ range(0.0, 1.0, length = size(allmu,1))]
plot([1.0],[1.0],grid=:true)
scatter!(allmu[:,1].+derror[1],allmu[:,2].+derror[2],color = my_colors[1:101],legend = :none)
savefig("simple_kf_2.png")
