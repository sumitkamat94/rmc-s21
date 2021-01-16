function cost(h1::Array{Float64,2},h2::Array{Float64,2})
d1 = sqrt((h1[1,4]-h2[1,4])^2 + (h1[2,4]-h2[2,4])^2+ (h1[3,4]-h2[3,4])^2);
d2 = transpose(h1[1:3,1:3])*h2[1:3,1:3]
d3 = (3.0- (d2[1,1]+d2[2,2]+d2[3,3]));
return d1+d3
end
