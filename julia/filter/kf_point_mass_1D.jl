# Define linear model for 1D point-mass system
T = 0.1 # sampling time
Ad = [1 T;0.0 1.0];
Bd = zeros(Float64,2,1);
Bd[1,1] = 0;
Bd[2,1]  =T;
Cd = [1.0 0.0]; # defines what we measure
# Cd = [0 1] => measure velocity
# Cd = [1 0] => measure position
Rv = diagm([5.0]) # Measurement noice variance
Rw = diagm([0.05;0.02]) # Process noise covariance
mysys = LTIsys(Ad,Bd,Cd,Rw,Rv)
Kgain = [-1 -1.0]

# initialize a plot
plot([1.0],[1.0],plot_title="plot_title")
println("eigenvalues: ",eigvals(Ad+(Bd*Kgain)))

# initialize true state and distribution
global mu = [0.0 ; 0.0];
global Sigma = diagm([100.0;10.0]);
global xTrue = [50;10.0]
add_plot_ellipse(mu[1],mu[2],Sigma,0)

anim = Plots.Animation()
scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)
Plots.frame(anim)

# begin simulation loop
ntimes = 100
sol = zeros(ntimes,4)
for i in 1:ntimes
  print(i,",")
  global mu
  global Sigma
  global xTrue
  uk = Kgain*mu;
  xTrue = Ad*xTrue + Bd*uk+Rw*rand(2,1);
  scatter!([xTrue[1]],[xTrue[2]],color = :green,legend = :none)
  Plots.frame(anim)
  yk = Cd*xTrue + Rv*randn(1,1);
  scatter!([yk],[xTrue[2]],color = :orange,legend = :none)
  Plots.frame(anim)
  mu,Sigma, mu_pred, Sigma_pred = filter_update(mysys,mu,Sigma,uk,yk);
  #println("mean:", mu)
  #println("true:",xTrue)
  sol[i,:] =transpose([mu;xTrue])
  add_plot_ellipse(mu_pred[1],mu_pred[2],Sigma_pred,0)
  Plots.frame(anim)
  add_plot_ellipse(mu[1],mu[2],Sigma,1)
  Plots.frame(anim)
end
gif(anim, "slam_oneD_fps20_5.gif", fps = 20)
savefig("oneD_kf.png")
#plot(sol)
# plot!([1.0],[1.0])

println("mean: ",mu[1:2])
println("truth: ",xTrue)
