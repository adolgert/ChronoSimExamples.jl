# Derived-generator twin of ReliabilitySim (src/reliability/reliability.jl).
#
# Same physical state, events, enable/fire!/rates. The difference: worker-event
# preconditions (EndDay, Break, Repair) are declared with `@precondition`, which
# derives their place-triggered generators from the precondition body — the
# hand-written `@conditionsfor` blocks for those events are deleted. StartDay keeps
# its manual generators because its precondition is `true` (reads no physical
# state), which is out of the derivable fragment.
#
# Distinct struct types (parallel to ReliabilitySim) are required: generators() is
# one method per event type. clock_key uses nameof(type), so a derived EndDay and a
# hand-written EndDay share the same clock key — differential trajectories compare.
module ReliabilityDerivedSim
using ChronoSim
using ChronoSim.ObservedState
using CompetingClocks
using CompetingClocks: CombinedNextReaction
import ChronoSim: generators, precondition, enable, reenable, fire!
using Distributions
using Random

export run_reliability_derived

@enum Activity ready working broken

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

@observedphysical IndividualState begin
    actors::ObservedVector{Individual,Member}
    params::Vector{IndividualParams}
    workers_max::Int
    start_time::Float64
end

function IndividualState(actor_cnt, crew_size)
    done_rate = LogUniform(.8, 0.99)
    break_rate = LogNormal(1.5, 0.4)
    repair_rate = Weibull(1.0, 2.0)
    p = IndividualParams(done_rate, break_rate, repair_rate)
    params = Vector{IndividualParams}(undef, actor_cnt)
    fill!(params, p)
    actors = ObservedArray{Individual,Member}(undef, actor_cnt)
    for i in 1:actor_cnt
        actors[i] = Individual(ready, 0.0, 0.0)
    end
    return IndividualState(actors, params, crew_size, 8.0 / 24.0)
end

worker_cnt(physical::IndividualState) = length(physical.actors)

struct StartDay <: SimEvent end

# stays manual: precondition is `true`, so it reads no physical state and
# @precondition cannot derive a trigger from it.
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

@fire function fire!(evt::StartDay, physical, when, rng)
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

# Derived: the single clean read actors[evt.actor_idx].state binds actor_idx, so
# the trigger [actors, ℤ, state] is equivalent to the hand-written
# `changed(actors[actor].state) -> EndDay(actor)`. No @domain needed.
@precondition function precondition(evt::EndDay, physical)
    return physical.actors[evt.actor_idx].state == working
end

function enable(evt::EndDay, physical, when)
    return (physical.params[evt.actor_idx].done_dist, when)
end

@fire function fire!(evt::EndDay, physical, when, rng)
    physical.actors[evt.actor_idx].state = ready
    started_work = physical.actors[evt.actor_idx].started_working_time
    physical.actors[evt.actor_idx].work_age += when - started_work
end

struct Break <: SimEvent
    actor_idx::Int
end

@precondition function precondition(evt::Break, physical)
    return physical.actors[evt.actor_idx].state == working
end

function enable(evt::Break, physical, when)
    started_ago = when - physical.actors[evt.actor_idx].work_age
    @debug "Break enable $(started_ago) $when"
    return (physical.params[evt.actor_idx].fail_dist, started_ago)
end

@fire function fire!(evt::Break, physical, when, rng)
    physical.actors[evt.actor_idx].state = broken
    started_work = physical.actors[evt.actor_idx].started_working_time
    physical.actors[evt.actor_idx].work_age += when - started_work
end

struct Repair <: SimEvent
    actor_idx::Int
end

@precondition function precondition(evt::Repair, physical)
    return physical.actors[evt.actor_idx].state == broken
end

function enable(evt::Repair, physical, when)
    return (physical.params[evt.actor_idx].repair_dist, when)
end

@fire function fire!(evt::Repair, physical, when, rng)
    physical.actors[evt.actor_idx].state = ready
    physical.actors[evt.actor_idx].work_age = 0.0
end

function initialize!(physical::PhysicalState, rng)
    for idx in eachindex(physical.actors)
        physical.actors[idx].work_age = rand(rng, Uniform(0, 10))
        physical.actors[idx].state = ready
    end
end

function run_reliability_derived(days)
    agent_cnt = 15
    Sampler = CombinedNextReaction{Tuple,Float64}
    physical = IndividualState(agent_cnt, 10)
    included_transitions = [StartDay, EndDay, Break, Repair]
    sim = SimulationFSM(
        physical,
        included_transitions;
        rng=Xoshiro(2947223),
        sampler=Sampler()
    )
    initializer = function(init_physical, when, rng)
        initialize!(init_physical, rng)
    end
    stop_condition = function(physical, step_idx, event, when)
        return when > days
    end
    ChronoSim.run(sim, initializer, stop_condition)
    return sim.when
end

end
