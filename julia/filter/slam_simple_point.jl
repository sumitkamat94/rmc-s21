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
        Rv[(2*i-1):(2*i),(2*i-1):(2*i)] = Rvm;
        #Rw[(2+2*i-1):(2+2*i),(2+2*i-1):(2+2*i)] = 0.1*Rwm;
        C[2*i-1,1] = -1.0;
        C[2*i,2] = -1.0;
        C[2*i-1,2+2*i-1] = 1.0;
        C[2*i  ,2+2*i] = 1.0;
    end
    mysys = LTIsys(A,B,C,Rw,Rv)
    return mysys
end

# Start
Rv = diagm([0.003;0.005]) # range, bearing measurement noise
Rw = diagm([0.4;0.5]) # x, y  process noise
lmarks = [1.0 2.0; 2.0 4.0; -1.0 3.0; -1 -2]
nl = size(lmarks,1)
gr()
plot([1.0],[1.0])


# initialize robot
global mu = zeros(2+2*nl);
global Sigma = diagm(15*ones(2+2*nl));
global xTrue = [4.0;4.0]

# Get model
mysys = linearizePointModel(xTrue,Rv,Rw,0.0,lmarks);

# This loop simulates both real world and agent's imagined world
# Integrator is Euler, unlike true ODE simulators
for i in 1:1000
  global mu
  global Sigma
  global xTrue
  # Construct control, adding process noise
  v = rand(1)[1]*0.5 - xTrue[1];
  w = randn(1)[1]*0.5- xTrue[2];
  uk = [v;w];
  # Update true model in response to noisy input
  xTrue = mysys.A[1:2,1:2]*xTrue + uk + 1.0*Rw*randn(2,1);
  # Get a true measurement from all landmarks
  yk = zeros(2*nl);
  for j in 1:nl
    yk[2*j-1] = lmarks[j,1] - xTrue[1]
    yk[2*j] = lmarks[j,2] - xTrue[2]
  end
  # Add measurement noiose
  yk = yk+mysys.Rv*randn(2*nl,1)
  # Update the filter based on commanded control and measurement
  mu,Sigma = filter_update(mysys,mu,Sigma,uk,yk);
end

# Post-processing
derror = -mu[3:4]+lmarks[1,:]
#derror = -mu[1:2]+xTrue
println("true mapping error: ",derror)
println("relative location error: ",xTrue -mu[1:2] - derror)
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
