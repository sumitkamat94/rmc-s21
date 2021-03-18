

i_c1 = [1 0 0;
       0 0.083 0
       0 0 1];

i_c2 = [1 0 0;
       0 0.083 0
       0 0 1];

i_c3 = [1 0 0;
       0 0.33 0
       0 0 1];


m1 = 1
m2 = 1
m3 = 1


l_l1 = 0.4
l_l2 = 1.1
l_l3 = 1.1


r_l1 = 0.2
r_l2 = 0.05
r_l3 = 0.05



B = [1 0 0;
     0 1 0;
     0 0 1]

T_w_l1(q) = [cos(q) -sin(q) 0 0;
            sin(q) cos(q) 0 0;
            0 0 1 0;
            0 0 0 1]

T_w_c1(q) = T_w_l1(q) * [cos(q) -sin(q) 0 0;
                         sin(q) cos(q) 0 0;
                         0 0 1 (l_l1/2.0);
                         0 0 0 1]

T_w_l2(q1, q2) = T_w_c1(q1) *  [cos(q2) -sin(q2) 0 0;
                                0 0 1 0;
                                -sin(q2) -cos(q2) 0 (l_l1/2.0)-0.05;
                                0 0 0 1]

T_w_c2(q1, q2) = T_w_l2(q1, q2) * [1 0 0 0;
                                   0 1 0 -(l_l2/2.0);
                                   0 0 1 0
                                   0 0 0 1]

T_w_l3(q1, q2, q3) = T_w_c2(q1, q2) * [cos(q3) -sin(q3) 0 0;
                                       sin(q3) cos(q3) 0 -(l_l2/2.0)+0.05;
                                       0 0 1 0;
                                       0 0 0 1]

T_w_c3(q1, q2, q3) = T_w_l3(q1, q2, q3) * [1 0 0 0;
                                           0 1 0 -(l_l3/2.0);
                                           0 0 1 (r_l2+r_l3);
                                           0 0 0 1]


function rot_c1(q)
    t = T_w_c1(q)
    return [t[1:3] t[5:7] t[9:11]]
end

function rot_c2(q1, q2)
    t = T_w_c2(q1,q2)
    return [t[1:3] t[5:7] t[9:11]]
end

function rot_c3(q1, q2, q3)
    t = T_w_c3(q1,q2,q3)
    return [t[1:3] t[5:7] t[9:11]]
end



function I_c1(q)
    r = rot_c1(q)
    return r*i_c1*r'
end

function I_c2(q1, q2)
    r = rot_c2(q1, q2)
    return r*i_c2*r'
end

function I_c3(q1, q2, q3)
    r = rot_c3(q1, q2, q3)
    return r*i_c3*r'
end



function Jc1(q)
    c1 = T_w_c1(q)

    T = T_w_l1(q)

    w = T[9:11]
    v = cross(T[9:11],(c1[13:15]-T[13:15]))

    return [[w; v] [0; 0; 0; 0; 0; 0] [0; 0; 0; 0; 0; 0]]

end

function Jc2(q1, q2)
    c2 = T_w_c2(q1,q2)

    T0 = T_w_l1(q1)
    T1 = T_w_l2(q1,q2)

    w1 = T0[9:11]
    w2 = T1[9:11]

    v1 = cross(T0[9:11],(c2[13:15]-T0[13:15]))
    v2 = cross(T1[9:11],(c2[13:15]-T1[13:15]))


    return [[w1; v1] [w2; v2] [0; 0; 0; 0; 0; 0]]

end

function Jc3(q1, q2, q3)
    c3 = T_w_c3(q1,q2,q3)

    T0 = T_w_l1(q1)
    T1 = T_w_l2(q1,q2)
    T2 = T_w_l3(q1,q2,q3)


    w1 = T0[9:11]
    w2 = T1[9:11]
    w3 = T2[9:11]

    v1 = cross(T0[9:11],(c3[13:15]-T0[13:15]))
    v2 = cross(T1[9:11],(c3[13:15]-T1[13:15]))
    v3 = cross(T2[9:11],(c3[13:15]-T2[13:15]))


    return [[w1; v1] [w2; v2] [w3; v3]]

end



function Jw(J)
    return [J[1:3] J[7:9] J[13:15]]
end

function Jv(J)
    return [J[4:6] J[10:12] J[16:18]]
end




function D(q)

    jc1 = Jc1(q[1])
    jc2 = Jc2(q[1],q[2])
    jc3 = Jc3(q[1],q[2],q[3])


    jwc1 = Jw(jc1)
    jvc1 = Jv(jc1)

    jwc2 = Jw(jc2)
    jvc2 = Jv(jc2)

    jwc3 = Jw(jc3)
    jvc3 = Jv(jc3)


    Ic1 = I_c1(q[1])
    Ic2 = I_c2(q[1],q[2])
    Ic3 = I_c3(q[1],q[2],q[3])

    D = (m1*jvc1'*jvc1 + jwc1'*Ic1*jwc1) +
        (m2*jvc2'*jvc2 + jwc2'*Ic2*jwc2) +
        (m3*jvc3'*jvc3 + jwc3'*Ic3*jwc3)

    return D

end


function C(q, q_dot)

    q1 = q[1]
    q2 = q[2]
    q3 = q[3]

    q1_dot = q_dot[1]
    q2_dot = q_dot[2]
    q3_dot = q_dot[3]

    # solved via Matlab Symbolic Toolbox (see find_coriolis.m)

    c11 = (1161*q2_dot*sin(2*q2))/1000 + (389*q2_dot*sin(2*q2 + 2*q3))/800 + (389*q3_dot*sin(2*q2 + 2*q3))/800 - (231*q3_dot*sin(q3))/800 + (231*q2_dot*sin(2*q2 + q3))/400 + (231*q3_dot*sin(2*q2 + q3))/800
    c12 = (1161*q1_dot*sin(2*q2))/1000 + (389*q1_dot*sin(2*q2 + 2*q3))/800 + (11*q2_dot*sin(q2 + q3))/200 + (11*q3_dot*sin(q2 + q3))/200 + (21*q2_dot*sin(q2))/200 + (231*q1_dot*sin(2*q2 + q3))/400
    c13 = (389*q1_dot*sin(2*q2 + 2*q3))/800 + (11*q2_dot*sin(q2 + q3))/200 + (11*q3_dot*sin(q2 + q3))/200 - (231*q1_dot*sin(q3))/800 + (231*q1_dot*sin(2*q2 + q3))/800
    c21 = -(q1_dot*(2310*sin(2*q2 + q3) + 4644*sin(2*q2) + 1945*sin(2*q2 + 2*q3)))/4000
    c22 = -(231*q3_dot*sin(q3))/400
    c23 = -(231*sin(q3)*(q2_dot + q3_dot))/400
    c31 = -(q1_dot*(231*sin(2*q2 + q3) + 389*sin(2*q2 + 2*q3) - 231*sin(q3)))/800
    c32 = (231*q2_dot*sin(q3))/400
    c33 = 0


    C = [c11 c12 c13;
         c21 c22 c23;
         c31 c32 c33]

    return C

end




function G(q)
    # i think g should be negative?
    g = -9.8
    return [0; -(1.6*sin(q[2])+0.55sin(q[2]+q[3]))*g; -0.55*sin(q[2]+q[3])*g]
end




function custom_inversedynamics(qd_dot, q_dot, q)

    torques = D(q)*qd_dot + C(q,q_dot)*q_dot + G(q) + B*q_dot

    return torques

end
