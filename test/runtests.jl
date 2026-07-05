using ChronoSimExamples
using Test

continuous_integration() = get(ENV, "CI", "false") == "true"

include("test_elevator.jl")
include("test_reliability.jl")
include("test_sirvillage.jl")
include("test_differential.jl")
include("test_invariants.jl")
include("test_overapprox.jl")
include("test_replay_examples.jl")
include("test_skeleton.jl")
