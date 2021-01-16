import Pkg;
Pkg.activate(@__DIR__);
using LinearAlgebra, Plots, Random
using MeshCat, GeometryTypes, CoordinateTransformations, FileIO, MeshIO
import rrt6d
vis2 = Visualizer();open(vis2)
