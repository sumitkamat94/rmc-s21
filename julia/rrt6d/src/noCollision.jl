function insidebox(p,xw,yw,zw)
  b = Array{Bool,1}(undef,3)
  b[1] = (p[1]<xw) && (-p[1]<xw)
  b[2] = (p[2]<yw) && (-p[2]<yw)
  b[3] = (p[3]<zw) && (-p[3]<zw)
  return (b[1] && b[2] && b[3])
end

function outsidebox(p,xw,yw,zw)
  b1 = all(x-> x > xw,p[1,:]) || all(x-> x >  yw,p[2,:]) || all(x-> x >  zw,p[3,:])
  b2 = all(x-> x <-xw,p[1,:]) || all(x-> x < -yw,p[2,:]) || all(x-> x < -zw,p[3,:])
  return b1 || b2
end

function get_box_corners(c,xw,yw,zw)
  # assumes c is an array of 3 numbers
  corner_points =
  [xw yw zw;
  xw -yw zw;
  xw yw -zw;
  xw -yw -zw;
  -xw yw zw;
  -xw -yw zw;
  -xw yw -zw;
  -xw -yw -zw]
  return corner_points+ones(8,1)*[c[1] c[2] c[3]]
end

function noCollision(h2, h1, obsall)
  # hard coding dimensions of robot: 1.5, 1.0, 0.5
  # assume there's a collision
  nc = true;

  # Combine h2 and h1 into a new frame
  o1 = 0.5*h1[1:3,4]+0.5*h2[1:3,4]
  v1 = h1[1:3,4]-o1
  edgeLength = norm(v1)
  v1 = v1/edgeLength;
  v1mat = reshape(v1,1,3)
  v2 = nullspace(v1mat);
  if size(v2,2) != 2
    nc = false
    return nc
  end
  Redge = [v1 v2];
  if det(Redge) < -0.1
      Redge = [v1 v2[:,2] v2[:,1]];
  end
  dedge = o1;

  # check for ground collision
  corner_points = get_box_corners([0.0,0.0,0.0],1.5+edgeLength,1.5,1.5)
  p = Redge*transpose(corner_points) + dedge*ones(1,8)
  if any(x -> x < 0.0, p[3,:])
    nc = false
    return nc
  end

  for i in 1:size(obsall,1)
    obs=obsall[i,:]
    if obs[5] == 0.0
      c_sphere1 = transpose(Redge)*([obs[1],obs[2],obs[3]]-dedge)

      nc = !insidebox(c_sphere1,1.5+obs[4]+edgelength,1.5+obs[4],1.5+obs[4])
    else
      cyl_corners = get_box_corners([obs[1],obs[2],obs[3]/2.0],obs[4],obs[4],obs[3]/2.0)
      local_cyl_corners = transpose(Redge)*(transpose(cyl_corners) - dedge*ones(1,8))
      nc = outsidebox(local_cyl_corners,1.5+edgeLength,1.5,1.5)
    end
    if nc == false
      break
    end
  end
  return nc
end

function noCollision_ends(h2, h1, obsall)
  # hard coding dimensions of robot: 1.5, 1.0, 0.5
  # assume there's a collision
  nc = true;

  # check for ground collision
  corner_points = get_box_corners([0.0,0.0,0.0],1.5,1.0,0.5)
  p1 = h1[1:3,1:3]*transpose(corner_points) + h1[1:3,4]*ones(1,8)
  p2 = h2[1:3,1:3]*transpose(corner_points) + h2[1:3,4]*ones(1,8)
  if any(x -> x < 0.0, p1[3,:]) || any(x -> x < 0.0, p2[3,:])
    nc = false
  end

  for i in 1:size(obsall,1)
    obs=obsall[i,:]
    if obs[5] == 0.0
      c_sphere1 = transpose(h1[1:3,1:3])*([obs[1],obs[2],obs[3]]-h1[1:3,4])
      c_sphere2 = transpose(h2[1:3,1:3])*([obs[1],obs[2],obs[3]]-h2[1:3,4])

      nc = !any([insidebox(c_sphere1,1.5+obs[4],1.0+obs[4],0.5+obs[4]),insidebox(c_sphere2,1.5+obs[4],1.0+obs[4],0.5+obs[4])])
    else
      cyl_corners = get_box_corners([obs[1],obs[2],obs[3]/2.0],obs[4],obs[4],obs[3]/2.0)
      local_cyl_corners1 = transpose(h1[1:3,1:3])*(transpose(cyl_corners) - h1[1:3,4]*ones(1,8))
      local_cyl_corners2 = transpose(h2[1:3,1:3])*(transpose(cyl_corners) - h2[1:3,4]*ones(1,8))
      nc = outsidebox(local_cyl_corners1,1.5,1.0,0.5) && outsidebox(local_cyl_corners2,1.5,1.0,0.5)
    end
    if nc == false
      break
    end
  end
  return nc
end
