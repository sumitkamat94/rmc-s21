function ccw(A,B,C)
    return (C[2]-A[2]) * (B[1]-A[1]) > (B[2]-A[2]) * (C[1]-A[1]);
end
