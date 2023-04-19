import Pkg;
Pkg.activate(@__DIR__);
Pkg.instantiate()
Pkg.precompile()
using LinearAlgebra, Plots, Random
# object that is an LTI system
struct LTIsys
  A::Array{Float64,2}
  B::Array{Float64,2}
  C::Array{Float64,2}
  Rw::Array{Float64,2}
  Rv::Array{Float64,2}
end
# implements the basic KF update
include("filter_update.jl")
