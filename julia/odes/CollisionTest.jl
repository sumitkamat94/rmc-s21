# using Pkg
# Pkg.activate(@__DIR__);
#
# pkg"instantiate"
# pkg"precompile"
#
# using RigidBodyDynamics, MeshCat, MeshCatMechanisms
# using LinearAlgebra

delete!(vis)
mechanism=parse_urdf("bed_part.urdf")
remove_fixed_tree_joints!(mechanism)
mvis = MechanismVisualizer(mechanism, URDFVisuals("bed_part.urdf"),vis)

hs_p = Point3D(root_frame(mechanism),0.0,0.0,0.0)
hs_v = FreeVector3D(root_frame(mechanism),0.0,0.2,1.0)
hs = RigidBodyDynamics.Contact.HalfSpace3D(hs_p,hs_v)
ce = RigidBodyDynamics.Contact.ContactEnvironment{Float64}()
push!(ce.halfspaces,hs)
mechanism.environment = ce

import RigidBodyDynamics.Contact.ViscoelasticCoulombModel
import RigidBodyDynamics.Contact.HuntCrossleyModel
import RigidBodyDynamics.Contact.SoftContactModel
conmod1 = HuntCrossleyModel(50e3,1.5*0.2*50e3,1.5)
conmod2 = ViscoelasticCoulombModel(0.1,10e5,10e3)
conmod3 = SoftContactModel(conmod1,conmod2)
#conmod3 = SoftContactModel(0,0)
cp1 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),1.0,1.0,1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp1)
cp2 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),-1.0,-1.0,-1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp2)
cp3 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),1.0,1.0,-1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp3)
cp4 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),1.0,-1.0,-1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp4)
cp5 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),1.0,-1.0,1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp5)
cp6 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),-1.0,1.0,-1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp6)
cp7 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),-1.0,-1.0,1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp7)
cp8 = RigidBodyDynamics.Contact.ContactPoint(Point3D(default_frame(bodies(mechanism)[2]),-1.0,1.0,1.0),conmod3)
add_contact_point!(bodies(mechanism)[2],cp8)





state = MechanismState(mechanism)
zero_velocity!(state)
set_configuration!(state,[1,0,0.0,0.0,0,0,1.0])

final_time = 1
ts, qs, vs = simulate(state, final_time);

# mvis = MechanismVisualizer(mechanism, URDFVisuals("bed_part.urdf"))
setanimation!(mvis,ts,qs)
# open(mvis);

#MeshCatMechanisms.animate(mvis, ts, qs; realtimerate = 1.);
