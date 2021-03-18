mech_path = path(mechanism,bodies(mechanism)[1],bodies(mechanism)[end])
Jq = geometric_jacobian(state,mech_path)
println(Jq.angular)
println(Jq.linear)
