
vis = Visualizer();
mvis, mechanism = display_urdf("planar3R.urdf",vis)
state = MechanismState(mechanism)

using Blink, Interact

function setVisConfig(q,state,mvis)
    set_configuration!(state,q)
    set_configuration!(mvis, configuration(state))
end 
function calcXinW(state,x)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[end])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,base_frame,ee_frame)
    x_in_w = inv(EEinBase)*x
    println("x in world: $((x_in_w).v)")
    # x_in_ee = EEinBase * x_in_w
    # println("x in block: $x_in_ee")

    or_w = Point3D(ee_frame, 0.0, 0.0, 0.0);
    x1_w = inv(EEinBase)*(Point3D(ee_frame, 1.0, 0.0, 0.0) - or_w)
    x2_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 1.0, 0.0) - or_w)
    x3_w = inv(EEinBase)*(Point3D(ee_frame, 0.0, 0.0, 1.0) - or_w)
    R = hcat(x1_w.v,x2_w.v,x3_w.v)
    x_in_w_manual = R *x.v + (inv(EEinBase)*Point3D(ee_frame, 0.0, 0.0, 0.0)).v
    println("x in world manual: $x_in_w_manual")
end
function calcOrinW(state,framenum)
    mechanism = state.mechanism;
    ee_frame=default_frame(bodies(mechanism)[framenum])
    base_frame = default_frame(bodies(mechanism)[1])
    # Obtain the transformation from world to EE
    EEinBase = relative_transform(state,ee_frame,base_frame)
    or_in_ee = Point3D(ee_frame, 0.0, 0.0, 0.0);
    or_in_w = EEinBase*or_in_ee
    # return "origin of frame $framenum in world: $or_in_w"
    return translation(EEinBase), rotation(EEinBase)
end
"
    linkTransformation(q,L)
The transformation between successive links in planar3R.urdf
q: angle of joint i
L: distance along frame x_{i-1} of origin of frame i
theta: angle of frame x w.r.t frame x_{i-1}
"
function linkTransformation(q,L)
    ## Assumes are child links have unrotated default frame 
    ## and child origins are on the $x$ axis at distance $L$
    return [cos(q) -sin(q) 0 L; 
         sin(q)  cos(q) 0 0; 
         0           0          1 0; 
         0           0          0 1]
end
function frameToBase(state,framenum)
    ## vector of link lengths: 
    if framenum < 3 
        return [0,0,0,0]
    end
    L = [0,1,1,1];
    q = vcat(configuration(state),0.0)
    ## Assumes all angles are zero 
    x0 = [0,0,0,1];
    for i=framenum-2:-1:1
        
        x0 = linkTransformation(q[i],L[i])*x0
        # println(q[i]," ",L[i], " ",x0)
    end
    return x0
end

## Define display areas 
text1 = textarea("")
text2 = textarea("")
text3 = textarea("")
text4 = textarea("")
## Define interactive elements and the callback effects
frames = OrderedDict(zip(map(x->x.name,bodies(mechanism)), [1, 2, 3, 4, 5, 6]))
mp = @manipulate for q1 in slider(-1.6:0.01:1.6; label="q1"), q2 in slider(-1.6:0.01:1.6; label="q2"), q3 in slider(-1.6:0.01:1.6; label="q3"), framenum in frames
    set_configuration!(state,[q1;q2;q3])
    set_configuration!(mvis, configuration(state))
    d, R = calcOrinW(state,framenum);
    d = round.(d,digits=3);
    text1[] = "x: "*string(d[1])*"\ny: "*string(d[2])*"\nz: "*string(d[3]);
    text2[] = string(round.(R[1,:],digits=2))*"\n"*string(string(round.(R[2,:],digits=2)))*"\n"*string(string(round.(R[3,:],digits=2)));
    d_calc = round.(frameToBase(state,framenum),digits=3)
    text3[] = "x: "*string(d_calc[1])*"\ny: "*string(d_calc[2])*"\nz: "*string(d_calc[3]);
end
## Create window
w1 = Window();
## Define layout of elements
ui = vbox(mp,
hbox(vbox("RigidBody functions",hbox(text1,text2)), vbox( "manual calculation",hbox(text3,text4))),
vis );  
## Push layout into window 
body!(w1,ui);
## For a separate window containing MeshCat visualizer
# w2 = Window();open(vis,w2);
