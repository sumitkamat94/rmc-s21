
# Obtain the end-effector frame and world frame of robot
ee = bodies(mechanism)[6]
ee_frame=default_frame(ee)
world_frame = root_frame(mechanism)
# Define point
x = Point3D(ee_frame, 0.2, 0.2, 0.2)
setelement!(mvis, ee_frame)
rad=0.05; # radius of point marker sphere
name = "p1"
setelement!(mvis,x, rad, name)

# Obtain the transformation from world to EE
EEinBase = relative_transform(state,world_frame,ee_frame)
x_in_w = inv(EEinBase)*x
x_in_ee = EEinBase * x_in_w
