mvis, pandamech = display_urdf("panda.urdf",vis)
state = MechanismState(pandamech)

using Blink, Interact
mp = @manipulate for q1 in slider(0:0.05:1; label="q1"), q2 in slider(0:0.05:1; label="q2"), q3 in slider(0:0.05:1; label="q3"), q4 in slider(0:0.05:1; label="q4"), q5 in slider(0:0.05:1; label="q5"), q6 in slider(0:0.05:1; label="q6"), q7 in slider(0:0.05:1; label="q7"), q8 in slider(0:0.05:1; label="q8")
    set_configuration!(state,[q1;q2;q4;q4;q5;q6;q7;q8;q8])
    set_configuration!(mvis, configuration(state))
    # println(q1,q2)
    # hbox(vis)
end
# open(vis,Blink.Window())
w = Window()
body!(w,mp)

