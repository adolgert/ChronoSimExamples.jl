using ChronoSimExamples
using Test

continuous_integration() = get(ENV, "CI", "false") == "true"

include("test_elevator.jl")
include("test_reliability.jl")
