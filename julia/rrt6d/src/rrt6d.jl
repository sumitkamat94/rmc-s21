module rrt6d

export steer
export dist
export cost
export noCollision
export sampleh

using MeshCat, GeometryTypes, CoordinateTransformations, FileIO, MeshIO, ColorTypes, LinearAlgebra

struct FrameHP
  h::Array{Float64,2}
  cost::Float64
  parent::Int
end


greet() = print("Hello World!")
include("steer.jl")
include("dist.jl")
include("cost.jl")
include("noCollision.jl")
include("ccw.jl")
include("utils.jl")
end # module
