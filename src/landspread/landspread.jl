module LandSpread

using ChronoSim
using CompetingClocks
import ChronoSim: generators, precondition, enable, reenable, fire!
using ChronoSim.ObservedState
using Distributions
using Random

# This will simulate spread across a landscape as a process on a graph.
# There are points on a 2D plane and distances among those points.
# The rate of spread from one point to the next depends on the distance.
# You start it somewhere and observe it later.

export run_landspread, landspread_likelihood

# We won't use the ChronoSim.@observedphysical macro for this simulation.
# Instead, every time we read from or write to the state, we'll note it to
# the simulation using the @obsread and @obswrite macros.
struct Landscape <: ObservedPhysical
    mark::Array{Int,1}  # State of each point.
    distance::Array{Float64,2}  # Distance matrix among points.
    obs_read::Set{Tuple}
    obs_modified::Set{Tuple}
end

function Landscape(N::Int, rng::AbstractRNG)
    locs = rand(rng, 2, N)
    dx = zeros(Float64, N, N)
    for i in 1:N, j in 1:N
        dx[j, i] = sqrt((locs[1, j] - locs[1, i])^2 + (locs[2, j] - locs[2, i])^2)
    end
    Landscape(zeros(Int, N), dx, Set{Tuple}(), Set{Tuple}())
end

struct Spread <: SimEvent
    source::Int
    destination::Int
end

@conditionsfor Spread begin
    @reactto changed(mark[i]) do land
        for j in 1:length(land.mark)
            generate(Spread(i, j))
        end
    end
end

@guard function precondition(evt::Spread, land)
    @obsread land.mark[evt.source]
    @obsread land.mark[evt.destination]
    return evt.source != evt.destination &&
        land.mark[evt.source] == 1 &&
        land.mark[evt.destination] == 0
end

# This module is written against ChronoSim's θ (parameter) seam, the
# four-argument form of `enable`: the simulation carries a parameter vector in
# its `params` field (constructor keyword `params=`), and the engine passes it
# to `enable` as the third argument. Reading the spread coefficients from θ
# instead of hard-coding them means an estimator can re-evaluate this same
# trace at a different θ — including a vector of ForwardDiff duals — through
# the `params=` keyword of `trace_likelihood`, with no module-global state.
# θ[1] is the scale coefficient and θ[2] the distance exponent of the Weibull
# spread clock. Modules that still define the three-argument
# `enable(evt, physical, when)` keep working: the four-argument default forwards
# to it (see ReliabilitySim for a deliberate example of that fallback).
const SPREAD_THETA = [0.1, 1.2]

function enable(evt::Spread, land, θ, when)
    dist = land.distance[evt.source, evt.destination]
    scale = θ[1] * dist^θ[2]
    return (Weibull(2, scale), when)
end

# NOTE the two-statement write idiom: the assignment happens as a plain
# statement and `@obswrite` is applied to the ACCESS, only to record the
# address. Passing the whole assignment to `@obswrite` looks natural but the
# macro's assignment form drops the right-hand side (it records the address and
# evaluates only the left-hand access), so the write never happens and the
# simulation silently proposes nothing. That bug made this example run
# zero events until the adoption audit caught it.
@fire function fire!(evt::Spread, land, when, rng)
    land.mark[evt.destination] = 1
    @obswrite land.mark[evt.destination]
end

function init_physical!(land, when, rng)
    land.mark[1] = 1
    @obswrite land.mark[1]
end

struct TrajectoryEntry
    event::Tuple
    when::Float64
end

astuple(te::TrajectoryEntry, event_dict) = (te.when, te.event)

mutable struct TrajectorySave
    trajectory::Vector{TrajectoryEntry}
    sim::SimulationFSM
    TrajectorySave() = new(Vector{TrajectoryEntry}())
end

function (te::TrajectorySave)(physical, when, event, changed_places)
    @info "Firing $event at $when"
    push!(te.trajectory, TrajectoryEntry(clock_key(event), when))
end

function run_landspread(point_cnt; θ=SPREAD_THETA)
    rng = Xoshiro(9437294723)
    land = Landscape(point_cnt, rng)
    trajectory = TrajectorySave()
    sim = SimulationFSM(
        land,
        [Spread];
        rng = rng,
        observer = trajectory,
        params = θ,
        )
    stop_condition = function(land, step, event, when)
        return false
    end
    ChronoSim.run(sim, init_physical!, stop_condition)
    return trajectory.trajectory
end


# Evaluate the recorded trajectory's log-likelihood at an explicit θ. Because θ
# arrives through the `params=` keyword rather than through anything the model
# module owns, the same trace can be scored at the θ that generated it, at a
# perturbed θ, or at a dual-valued θ for automatic differentiation — see the
# RepairShop module for that workflow taken through ForwardDiff.
function landspread_likelihood(point_cnt; θ=SPREAD_THETA)
    trajectory_vector = run_landspread(point_cnt)
    event_dict = Dict(:Spread => Spread, :InitializeEvent => ChronoSim.InitializeEvent)
    event_vector = [astuple(te, event_dict) for te in trajectory_vector]
    @assert event_vector[1][2][1] == :InitializeEvent
    popfirst!(event_vector)  # Get rid of the InitializeEvent.
    rng = Xoshiro(9437294723)
    land = Landscape(point_cnt, rng)
    sim = SimulationFSM(
        land,
        [Spread];
        sampler = NextReactionMethod(), key_type = Tuple,
        step_likelihood = true,
        likelihood_eltype = eltype(θ),
        rng = rng
        )
    how_likely = ChronoSim.trace_likelihood(
        sim, init_physical!, event_vector; params=θ
    ).loglikelihood
    println("logpdf $how_likely")
    return how_likely
end
end  # module LandSpread


if abspath(PROGRAM_FILE) == @__FILE__
    using .LandSpread
    run_landspread(10)
    landspread_likelihood(10)
end
