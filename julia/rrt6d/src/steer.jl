function steer(qr,qn,val,eps)
   qnew = zeros(Float64, 4, 4);
   # Steer towards qn with maximum step size of eps
   if val >= eps # if min distance is larger than EPS, interpolate
      rot = qn[1:3,1:3]*transpose(qr[1:3,1:3])
      d = dist(qr,qn)
      theta = acos((tr(rot)-1)/2.0)
      targettrace = 3.0 - eps/d/2.0*(3.0 -tr(rot))
      thetanew = acos((targettrace-1)/2.0)
      if theta == 0.0
          qnew[1:3,1:3] = qr[1:3,1:3]
      else
          w = (1/2/sin(theta))*[rot[3,2]-rot[2,3],rot[1,3]-rot[3,1],rot[2,1]-rot[1,2]]
          K = [0 -w[3] w[2]; w[3] 0.0 -w[1]; -w[2] w[1] 0.0 ]
          #thetanew = theta*eps/d/2.0
          deltarot = [1.0 0.0 0.0;0.0 1.0 0.0 ; 0.0 0.0 1.0] + sin(thetanew)*K + (1-cos(thetanew))*K*K
          qnew[1:3,1:3] = deltarot*qn[1:3,1:3]
      end

       qnew[1,4] = qn[1,4] + ((qr[1,4]-qn[1,4])*eps/2.0)/d;
       qnew[2,4] = qn[2,4] + ((qr[2,4]-qn[2,4])*eps/2.0)/d;
       qnew[3,4] = qn[3,4] + ((qr[3,4]-qn[3,4])*eps/2.0)/d;

   else # otherwise, q_rand itself is the accepted new point
       qnew = qr;
   end
   return qnew
end
