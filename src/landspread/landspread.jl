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
# the simulation using the @observe macro.
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

function precondition(evt::Spread, land)
    @observe land.mark[evt.source]
    @observe land.mark[evt.destination]
    return evt.source != evt.destination &&
        land.mark[evt.source] == 1 &&
        land.mark[evt.destination] == 0
end

function enable(evt::Spread, land, when)
    dist = land.distance[evt.source, evt.destination]
    scale = 0.1 * dist^1.2
    return (Weibull(2, scale), when)
end

function fire!(evt::Spread, land, when, rng)
    @observe land.mark[evt.destination] = 1
end

function init_physical!(land, when, rng)
    @observe land.mark[1] = 1
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

function run_landspread(point_cnt)
    rng = Xoshiro(9437294723)
    land = Landscape(point_cnt, rng)
    trajectory = TrajectorySave()
    sim = SimulationFSM(
        land,
        [Spread];
        rng = rng,
        observer = trajectory,
        )
    stop_condition = function(land, step, event, when)
        return false
    end
    ChronoSim.run(sim, init_physical!, stop_condition)
    return trajectory.trajectory
end


function landspread_likelihood(point_cnt)
    trajectory_vector = run_landspread(point_cnt)
    event_dict = Dict(:Spread => Spread, :InitializeEvent => ChronoSim.InitializeEvent)
    event_vector = [astuple(te, event_dict) for te in trajectory_vector]
    @assert event_vector[1][2][1] == :InitializeEvent
    popfirst!(event_vector)  # Get rid of the InitializeEvent.
    rng = Xoshiro(9437294723)
    land = Landscape(point_cnt, rng)
    sampler = CompetingClocks.MemorySampler(CombinedNextReaction{Tuple,Float64}())
    sim = SimulationFSM(
        land,
        [Spread];
        sampler = sampler,
        rng = rng
        )
    how_likely = ChronoSim.trace_likelihood(sim, init_physical!, event_vector)
    println("logpdf $how_likely")
end
end  # module LandSpread


if abspath(PROGRAM_FILE) == @__FILE__
    using .LandSpread
    run_landspread(10)
    landspread_likelihood(10)
end
