module ReliabilitySim
using ChronoSim
using ChronoSim.ObservedState
using CompetingClocks
import ChronoSim: generators, precondition, enable, reenable, fire!
using Distributions
using Random

export run_reliability

@enum Activity ready working broken

# Using keyedby says the fields of this struct can be used to track events.
@keyedby Individual Int64 begin
    state::Activity
    work_age::Float64
    started_working_time::Float64
end

struct IndividualParams
    done_dist::LogUniform
    fail_dist::LogNormal
    repair_dist::Weibull
end

# This is the physical state of the system. It will report its reads and writes.
@observedphysical IndividualState begin
    actors::ObservedVector{Individual}
    params::Vector{IndividualParams}
    workers_max::Int
    start_time::Float64
end


function IndividualState(actor_cnt, crew_size)
    done_rate = LogUniform(.8, 0.99) # Gamma(9.0, 0.2)
    break_rate = LogNormal(1.5, 0.4)
    repair_rate = Weibull(1.0, 2.0)
    p = IndividualParams(done_rate, break_rate, repair_rate)
    params = Vector{IndividualParams}(undef, actor_cnt)
    fill!(params, p)
    actors = ObservedArray{Individual}(undef, actor_cnt)
    for i in 1:actor_cnt
        actors[i] = Individual(ready, 0.0, 0.0)
    end
    return IndividualState(actors, params, crew_size, 8.0 / 24.0)
end

worker_cnt(physical::IndividualState) = length(physical.actors)

struct StartDay <: SimEvent end

precondition(event::StartDay, physical) = true

@conditionsfor StartDay begin
    @reactto changed(actors[i].state) do physical
        generate(StartDay())
    end
end

function enable(evt::StartDay, physical, when)
    desired_time = floor(when) + 1.0 + physical.start_time
    interval = desired_time - when
    return (Dirac(interval), when)
end

function fire!(evt::StartDay, physical, when, rng)
    crew_cnt = 0
    for car in eachindex(physical.actors)
        if physical.actors[car].state == ready
            physical.actors[car].state = working
            physical.actors[car].started_working_time = when
            crew_cnt += 1
            if crew_cnt == physical.workers_max
                break
            end
        end
    end
end

struct EndDay <: SimEvent
    actor_idx::Int
end

precondition(evt::EndDay, physical) = physical.actors[evt.actor_idx].state == working

@conditionsfor EndDay begin
    @reactto changed(actors[actor].state) do physical
        generate(EndDay(actor))
    end
end

function enable(evt::EndDay, physical, when)
    return (physical.params[evt.actor_idx].done_dist, when)
end

function fire!(evt::EndDay, physical, when, rng)
    physical.actors[evt.actor_idx].state = ready
    started_work = physical.actors[evt.actor_idx].started_working_time
    physical.actors[evt.actor_idx].work_age += when - started_work
end


struct Break <: SimEvent
    actor_idx::Int
end

precondition(evt::Break, physical) = physical.actors[evt.actor_idx].state == working

@conditionsfor Break begin
    @reactto changed(actors[actor].state) do physical
        generate(Break(actor))
    end
end

function enable(evt::Break, physical, when)
    started_ago = when - physical.actors[evt.actor_idx].work_age
    @debug "Break enable $(started_ago) $when"
    return (physical.params[evt.actor_idx].fail_dist, started_ago)
end

function fire!(evt::Break, physical, when, rng)
    physical.actors[evt.actor_idx].state = broken
    started_work = physical.actors[evt.actor_idx].started_working_time
    physical.actors[evt.actor_idx].work_age += when - started_work
end

struct Repair <: SimEvent
    actor_idx::Int
end

precondition(evt::Repair, physical) = physical.actors[evt.actor_idx].state == broken

@conditionsfor Repair begin
    @reactto changed(actors[actor].state) do physical
        generate(Repair(actor))
    end
end

function enable(evt::Repair, physical, when)
    return (physical.params[evt.actor_idx].repair_dist, when)
end

function fire!(evt::Repair, physical, when, rng)
    physical.actors[evt.actor_idx].state = ready
    physical.actors[evt.actor_idx].work_age = 0.0
end

function initialize!(physical::PhysicalState, rng)
    for idx in eachindex(physical.actors)
        # This is a warm start to the problem.
        physical.actors[idx].work_age = rand(rng, Uniform(0, 10))
        physical.actors[idx].state = ready
    end
end



struct TrajectoryEntry
    event::Tuple
    when::Float64
end

struct TrajectorySave
    trajectory::Vector{TrajectoryEntry}
    TrajectorySave() = new(Vector{TrajectoryEntry}())
end

function (ts::TrajectorySave)(physical, when, event, changed_places)
    @debug "Firing $event at $when"
    push!(ts.trajectory, TrajectoryEntry(clock_key(event), when))
end


function run_reliability(days)
    agent_cnt = 15
    Sampler = CombinedNextReaction{Tuple,Float64}
    physical = IndividualState(agent_cnt, 10)
    included_transitions = [
        StartDay,
        EndDay,
        Break,
        Repair
    ]
    trajectory = TrajectorySave()
    sim = SimulationFSM(
        physical,
        Sampler(),
        included_transitions;
        rng=Xoshiro(2947223),
        observer=trajectory
    )
    initializer = function(init_physical, when, rng)
        initialize!(init_physical, rng)
    end
    # Stop-condition is called after the next event is chosen but before the
    # next event is fired. This way you can stop at an end time between events.
    stop_condition = function(physical, step_idx, event, when)
        return when > days
    end
    ChronoSim.run(sim, initializer, stop_condition)
    return sim.when
end

end
