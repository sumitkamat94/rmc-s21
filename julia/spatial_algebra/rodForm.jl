# Rodrigues' formula
theta = 0.3454;
update_block_state(state,mvis,theta,w,d)
matexp(w) = [0 -w[3] w[2];w[3] 0 -w[1]; -w[2] w[1] 0 ];
Sofw = matexp(w/norm(w));
R1 = diagm([1.0,1.0,1.0]) + sin(theta)*Sofw  + (1-cos(theta))*Sofw*Sofw

# Compare to point-based construction for block
EEinBase = relative_transform(state,base_frame,ee_frame)
or_w = Point3D(ee_frame, 0.0, 0.0, 0.0);
x1_w = inv(EEinBase)*(Point3D(ee_frame, 1.0, 0.0, 0.0) - or_w)
x2_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 1.0, 0.0) - or_w)
x3_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 0.0, 1.0) - or_w)
R2 = hcat(x1_w.v,x2_w.v,x3_w.v)
