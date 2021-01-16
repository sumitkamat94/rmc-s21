function rotobjModel(xTrue,Rvm,Rwm,uk,lmarks)
# Linearizes the model at current state and input
    nl = size(lmarks,1)
    nstate = length(xTrue)
    # Initialize matrices
    A = zeros(nstate+2*nl,nstate+2*nl)
    B = zeros(nstate+2*nl,2)
    C = zeros(2*nl,nstate+2*nl)
    Rv = zeros(2*nl,2*nl)
    Rw = zeros(nstate+2*nl,nstate+2*nl)
    ctheta = cos(xTrue[3])
    stheta = sin(xTrue[3])

    # Start populating
    A[1,1] = 1.0;    A[1,2] = -stheta*uk[1];
    A[2,2] = 1.0;    A[1,3] =  ctheta*uk[1];
    A[3,3] = 1.0;
    B[1,1] = ctheta;
    B[2,1] = stheta;
    B[3,2] = 1.0;
    Rw[1:3,1:3] = Rwm
    RotMatT = [ctheta stheta;-stheta ctheta]
    for i in 1:nl
        A[nstate+2*i-1,nstate+2*i-1] = 1.0;
        A[nstate+2*i  ,nstate+2*i  ] = 1.0;
        Rv[(2*i-1):(2*i),(2*i-1):(2*i)] = Rvm;
        C[(2*i-1):(2*i),1:2] = -RotMatT
        C[(2*i-1),3] = stheta*xTrue[1]+ctheta*xTrue[2]
        C[(2*i),3] = -ctheta*xTrue[1]+stheta*xTrue[2]
        C[(2*i-1):(2*i),(nstate+2*i-1):(nstate+2*i)] = RotMatT
    end
    mysys = LTIsys(A,B,C,Rw,Rv)
    return mysys
end

# Start
# Define measurement+processnoise and landmarks
Rv = diagm([0.5;0.5]) # range, bearing measurement noise
Rw = diagm([0.004;0.005;0.002]) # x, y, theta process noise
lmarks = [1.0 2.0; 2.0 4.0; -1.0 3.0; -1 -2]
nl = size(lmarks,1)

gr() # call the GR plotting backend
plot([0.0],[0.0]) # dummy plot to clear figure


# initialize robot
global mu = zeros(3+2*nl);
global Sigma = diagm(10*ones(3+2*nl));
global xTrue = [randn(1)[1]*4.0;randn(1)[1]*4.0;randn(1)[1]*1.0]

# Simulate multiple steps
# Simulating both the real world
# And the robot's processing
for i in 1:1000
  global mu
  global Sigma
  global xTrue

  # choose a control
  v = randn(1)[1]*0.05;
  w = randn(1)[1]*0.05;
  uk = [v;w];

  # get model
  mysys1 = rotobjModel(xTrue,Rv,Rw,uk,lmarks);

  # Motion
  xTrue = xTrue + mysys1.B[1:3,:]*uk + 1.0*Rw*randn(3,1);

  # Measurement
  mysys2 = rotobjModel(xTrue,Rv,Rw,uk,lmarks);
  ctheta = cos(xTrue[3])
  stheta = sin(xTrue[3])
  yk = zeros(2*nl);

  # Update using linearized matrices from two states
  mysys = LTIsys(mysys1.A,mysys1.B,mysys2.C,mysys1.Rw,mysys2.Rv)
  for j in 1:nl
    yk[2*j-1] =  ctheta*(lmarks[j,1] - xTrue[1]) + stheta*(lmarks[j,2] - xTrue[2])
    yk[2*j]   = -stheta*(lmarks[j,1] - xTrue[1]) + ctheta*(lmarks[j,2] - xTrue[2])
  end
  yk = yk+mysys.Rv*randn(2*nl,1)
  # The next line is what the robot's `brain` does:
  mu,Sigma = filter_update(mysys,mu,Sigma,uk,yk);
end

# Plot the ground truth...
scatter!(lmarks[:,1],lmarks[:,2],color = :red,legend = :none)
scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)

# ...and the robot's belief, after aligning landmarks
derror = -mu[4:5]+lmarks[1,:]
println("relative location error: ",xTrue[1:2] -mu[1:2] - derror)

for j in 1:nl
  add_plot_ellipse(mu[3+2*j-1]+derror[1],mu[3+2*j]+derror[2],Sigma[(3+2*j-1):(3+2*j),(3+2*j-1):(3+2*j)],1)
end
add_plot_ellipse(mu[1]+derror[1],mu[2]+derror[2],Sigma[1:2,1:2],0)
scatter!([mu[1]+derror[1]],[mu[2]+derror[2]],color = :orange,legend = :none)
