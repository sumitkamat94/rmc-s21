mech_path = path(mechanism,bodies(mechanism)[1],bodies(mechanism)[end])
Jq = geometric_jacobian(state,mech_path)
println(Jq.angular)
println(Jq.linear)
x = Point3D(ee_frame, 1.0, 0.0, 0.0)
point_jacobian(state, mech_path, x)

function calcXinW(state,x)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[end])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,base_frame,ee_frame)
    x_in_w = inv(EEinBase)*x
    return x_in_w.v
end


q0 = [0.1,.1,0.1];
# X = [3.4,0.3,0.3]
for i=1:5000
    update_planar_state(state,mvis,q0[1],q0[2],q0[3])
    ## Get the Jacobian
    J = point_jacobian(state, mech_path, x).J[1:2,:]
    FKsol = vcat(calcXinW(state,x).data...)
    Jpi = hcat(J'*inv(J*J'),zeros(3))
    # println(FKsol)
    q0 .= q0 + 0.001*Jpi*(X - FKsol)
    # q0[1] = 0
end

calcXinW(state,x)