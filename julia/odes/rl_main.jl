


function custom_inversedynamics(q,qd,qdd)
  q1 = q[1,1]
  q2 = q[2,1]
  q3 = q[3,1]
  qd1 = qd[1,1]
  qd2 = qd[2,1]
  qd3 = qd[3,1]
  qdd1 = qdd[1,1]
  qdd2 = qdd[2,1]
  qdd3 = qdd[3,1]

  m = 1
  g = 9.81

  L1 = 0.5
  L2 = 1.1
  L3 = 1.1

  Ixx1 = 1
  Iyy1 = 0.083
  Izz1 = 1

  Ixx2 = 1
  Iyy2 = 0.083
  Izz2 = 1

  Ixx3 = 1
  Iyy3 = 0.33
  Izz3 = 1

  # Lagrange Equations

  T1 = Izz1*qdd1 - .5*qd2^2*(Ixx2*sin(q1)^2*cos(q2)^2 - Iyy2*sin(q1)^2 + Izz2*sin(q1)^2*sin(q2)^2) - .5*qd3^2*(Ixx3*sin(q1)^2*(cos(q2+q3)^2 - 2*cos(q2+q3)*sin(q2+q3) + sin(q2+q3)^2) - Iyy3*sin(q1)^2 + Izz3*sin(q1)^2*(cos(q1)^2*sin(q3)^2 + 2*cos(q2)*cos(q3)*sin(q2)*sin(q3) + sin(q2)^2*cos(q3)^2))

  T2 = qdd2*(Ixx2*sin(q1)^2*cos(q1)^2 + Iyy2*cos(q1)^2 + Izz2*sin(q1)^2*sin(q2)^2) - .5*qd2^2*(-Ixx2*sin(q1)^2*sin(q2)^2 + Izz2*sin(q1)^2*sin(q2)^2) - m*g*L2/2*sin(q2) - .5*qd3^2*(Ixx3*sin(q1)^2*(-sin(q2)^2 + 2*sin(q2)*cos(q2) + sin(q2)^2) + Izz3*sin(q1)^2*(-sin(q2)^2*sin(q3)^2 - 2*sin(q3)^2*cos(q3)^2 + sin(q2)^2*cos(q3)^2)) - m*g*L3/2*(sin(q2)*cos(q3) + cos(q2)*sin(q3))

  T3 = qdd3*(Ixx3*sin(q1)^2*(cos(q2+q3)^2 - 2*cos(q2+q3)*sin(q2+q3) + sin(q2+q3)^2) + Iyy3*cos(q1)^2 + Izz3*sin(q1)^2*(cos(q2)^2*sin(q3)^2 + 2*cos(q2)*sin(q2)*cos(q3)*sin(q3) + sin(q2)^2*cos(q3)^2)) - .5*qd3^2*(Ixx3*sin(q1)^2*(-sin(q3)^2 + 2*sin(q3)*cos(q3) + sin(q3)^2) + Izz3*sin(q1)^2*(cos(q2)^2*cos(q3)^2 - 2*cos(q2)*cos(q3)*sin(q2)*sin(q3) - sin(q2)^2*sin(q3)^2)) - m*g*L3/2*(-cos(q2)*sin(q3) - sin(q2)*cos(q3))

  T = [T1,T2,T3]

  return T

end

# User Input for Start and End point

print("Provide values for q, qdot, and qddot (rad, rad/s, rad/s^2 respectively) and the function will return the vector of joint torques required\n")
print("q1: \n")

input1 = readline()
input1 = parse(Float64, input1)

print("q2: \n")

input2 = readline()
input2 = parse(Float64, input2)

print("q3: \n")

input3 = readline()
input3 = parse(Float64, input3)

print("qdot1: \n")

input4 = readline()
input4 = parse(Float64, input4)

print("qdot2: \n")

input5 = readline()
input5 = parse(Float64, input5)

print("qdot3: \n")

input6 = readline()
input6 = parse(Float64, input6)

print("qddot1: \n")

input7 = readline()
input7 = parse(Float64, input7)

print("qddot2: \n")

input8 = readline()
input8 = parse(Float64, input8)

print("qddot3: \n")

input9 = readline()
input9 = parse(Float64, input9)

q = [input1,input2,input3]
qd = [input4,input5,input6]
qdd = [input7,input8,input9]

Torques = custom_inversedynamics(q,qd,qdd)


print("The torques corresponding to joint variables 1-3 respectively are:\n")
print(Torques)
