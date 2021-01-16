function steer(qr,qn,val,eps)
   qnew = [0.0 0.0];
   # Steer towards qn with maximum step size of eps
   if val >= eps # if min distance is larger than EPS, interpolate
       qnew[1] = qn[1] + ((qr[1]-qn[1])*eps)/dist(qr,qn);
       qnew[2] = qn[2] + ((qr[2]-qn[2])*eps)/dist(qr,qn);
   else # otherwise, q_rand itself is the accepted new point
       qnew[1] = qr[1];
       qnew[2] = qr[2];
   end
   A = [qnew[1] qnew[2]];
   return A
end
