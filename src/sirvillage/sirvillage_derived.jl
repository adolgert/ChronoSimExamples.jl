# Derived-generator twin of SIRVillage (src/sirvillage/sirvillage.jl).
#
# Same physical state, events, enable/fire!/rates. The difference: the in-fragment
# event preconditions (Infect, Recover, Reset, Mutate) are declared with
# `@precondition`, which derives their place-triggered generators from the
# precondition body — the hand-written `@conditionsfor` blocks for those events are
# deleted. Travel keeps its manual generator because its precondition is `true`
# (reads no physical state, rate-driven), which is out of the derivable fragment.
# InitEvent is the bootstrap event and has no precondition/generators in either twin.
#
# strains is an ObservedDict keyed by strain id with a next_strain_id counter, the
# same fixed-extent-safe change made in the hand-written twin (a born strain touches
# exactly one key).
#
# Distinct struct types (parallel to SIRVillage) are required: generators() is one
# method per event type. clock_key uses nameof(type), so a derived Infect and a
# hand-written Infect share the same clock key — differential trajectories compare.
module SIRVillageDerived

using CompetingClocks
using Distributions
using Logging
using PDMats
using Random

using ChronoSim
using ChronoSim.ObservedState
import ChronoSim: generators, precondition, enable, reenable, fire!

export run_sirvillage_derived

@enum DiseaseState Susceptible Infectious Recovered

@keyedby Individual Int64 begin
    state::DiseaseState
    strain::Int64
    haunt::Int64
end

struct IndividualParams
    robustness::Float64
    haunts::Vector{Int64}
    exit_sum::Vector{Float64}
    exit_frac::Vector{Float64}
end

@keyedby Location Int64 begin
    individual_cnt::Int64
    individuals::Vector{Int64}
end

@keyedby Strain Int64 begin
    infectivity::Float64
    virulence::Float64
    parent::Int64
end

# Same physical state as the hand-written twin, including the keyed strains dict and
# the next_strain_id counter (see the hand-written module for the WHY).
@observedphysical Village begin
    actors::ObservedVector{Individual,Member}
    # Param marks this immutable config UnObservable so Infect's precondition reads
    # of it emit no notification the coverage oracle would flag (see hand-written
    # twin for the full WHY).
    actor_params::Param{Vector{IndividualParams}}
    locations::ObservedVector{Location,Member}
    strains::ObservedDict{Int64,Strain,Member}
    next_strain_id::Int64
end

function movement_model(location_cnt, individual_location_cnt, day_length, rng)
    dwells = 0.1 * rand(rng, individual_location_cnt)
    if individual_location_cnt > 2
        dwells[1] = 0.4 * day_length
        dwells[2] = 0.3 * day_length
    end
    fractions = rand(rng, individual_location_cnt)
    fractions /= sum(fractions)
    exit_sum = @. (π / (2 * dwells^2))
    boosted_frac = fractions ./ dwells
    exit_frac = boosted_frac / sum(boosted_frac)
    locations = sample(rng, 1:location_cnt, individual_location_cnt; replace=false)
    @assert all(isfinite, exit_sum)
    @assert all(isfinite, exit_frac)
    return (locations, exit_sum, exit_frac)
end

function Village(actor_cnt::Int, location_cnt::Int, day_length::AbstractFloat, rng)
    robust_dist = Normal(1.0, 0.1)
    params = Vector{IndividualParams}(undef, actor_cnt)
    individual_locations = min(20, round(Int, 0.4 * location_cnt))
    for param_idx in eachindex(params)
        (locations, exit_sum, exit_frac) = movement_model(
            location_cnt, individual_locations, day_length, rng
            )
        params[param_idx] = IndividualParams(rand(rng, robust_dist), locations, exit_sum, exit_frac)
    end
    actors = ObservedArray{Individual,Member}(undef, actor_cnt)
    for i in 1:actor_cnt
        actors[i] = Individual(Susceptible, 0, 0)
    end
    locations = ObservedArray{Location,Member}(undef, location_cnt)
    for loc_idx in 1:location_cnt
        locations[loc_idx] = Location(0, Int64[])
    end
    strains = ObservedDict{Int64,Strain,Member}()
    initial_infectivity = 1.0
    initial_virulence = 1.0
    strains[1] = Strain(initial_infectivity, initial_virulence, 0)
    return Village(actors, params, locations, strains, 2)
end

struct Travel <: SimEvent
    who::Int64
end

# stays manual: reads no state (rate-driven). precondition is `true`, so
# @precondition cannot derive a trigger from it.
precondition(event::Travel, physical) = true

@conditionsfor Travel begin
    @reactto changed(actors[who].haunt) do physical
        generate(Travel(who))
    end
end

function enable(evt::Travel, physical, when)
    haunt_idx = physical.actors[evt.who].haunt
    scale = 2.0 / physical.actor_params[evt.who].exit_sum[haunt_idx]
    return (Weibull(2, scale), when)
end

@fire function fire!(evt::Travel, physical, when, rng)
    source_haunt = physical.actors[evt.who].haunt
    source_loc = physical.actor_params[evt.who].haunts[source_haunt]
    probs = physical.actor_params[evt.who].exit_frac
    mask = trues(length(probs))
    mask[source_haunt] = false
    probs_norm = probs / sum(probs[mask])
    probs_norm[source_haunt] = 0.0
    dest_haunt = rand(rng, Categorical(probs_norm))
    dest_loc = physical.actor_params[evt.who].haunts[dest_haunt]
    physical.locations[source_loc].individual_cnt -= 1
    filter!(w -> w != evt.who, physical.locations[source_loc].individuals)
    physical.actors[evt.who].haunt = dest_haunt
    physical.locations[dest_loc].individual_cnt += 1
    push!(physical.locations[dest_loc].individuals, evt.who)
end

struct Infect <: SimEvent
    source::Int64
    sink::Int64
end

# Derived: source binds from the clean reads actors[evt.source].state/.haunt and
# sink binds from actors[evt.sink].state/.haunt. A change to actors[i].state (or
# .haunt) therefore fires two derived generators — one placing i as source with sink
# free, one placing i as sink with source free — and each free field's domain is
# inferred as eachindex(actors) from the container-key of the other clean read, so
# no @domain is needed. This is the headline over-approximation: the hand-written
# generators propose only co-located pairs, the derived generators propose O(N) per
# state change, and the precondition (which requires source_loc == sink_loc) filters
# the derived proposals back to the same admitted set.
#
# The actor_params reads are on a plain (unobserved) Vector; the macro cannot tell
# syntactically, so it emits dead triggers for them that are never notified —
# harmless (over-approximation only) and invisible to the coverage oracle.
@precondition function precondition(evt::Infect, physical)
    source_state = physical.actors[evt.source].state
    source_haunt = physical.actors[evt.source].haunt
    source_loc = physical.actor_params[evt.source].haunts[source_haunt]
    sink_state = physical.actors[evt.sink].state
    sink_haunt = physical.actors[evt.sink].haunt
    sink_loc = physical.actor_params[evt.sink].haunts[sink_haunt]
    return (
        source_state == Infectious &&
        sink_state == Susceptible &&
        source_loc == sink_loc
        )
end

function enable(evt::Infect, physical, when)
    strain_idx = physical.actors[evt.source].strain
    infectivity = physical.strains[strain_idx].infectivity
    robust = physical.actor_params[evt.sink].robustness
    return (Exponential(inv(infectivity / robust)), when)
end

@fire function fire!(evt::Infect, physical, when, rng)
    strain_idx = physical.actors[evt.source].strain
    physical.actors[evt.sink].strain = strain_idx
    physical.actors[evt.sink].state = Infectious
end

struct Recover <: SimEvent
    who::Int64
end

# Derived: the single clean read actors[evt.who].state binds who, so the trigger
# [actors, ℤ, state] is equivalent to the hand-written
# `changed(actors[who].state) -> Recover(who)`. No @domain needed.
@precondition function precondition(evt::Recover, physical)
    return physical.actors[evt.who].state == Infectious
end

function enable(evt::Recover, physical, when)
    strain_idx = physical.actors[evt.who].strain
    virulence = physical.strains[strain_idx].virulence
    robust = physical.actor_params[evt.who].robustness
    return (Gamma(3, inv(robust / virulence)), when)
end

@fire function fire!(evt::Recover, physical, when, rng)
    physical.actors[evt.who].state = Recovered
end

struct Reset <: SimEvent
    who::Int64
end

# Derived: single clean read actors[evt.who].state binds who.
@precondition function precondition(evt::Reset, physical)
    return physical.actors[evt.who].state == Recovered
end

function enable(evt::Reset, physical, when)
    return (Exponential(inv(0.05)), when)
end

@fire function fire!(evt::Reset, physical, when, rng)
    physical.actors[evt.who].state = Susceptible
    physical.actors[evt.who].strain = 0
end

struct Mutate <: SimEvent
    carrier::Int64
end

# Derived: single clean read actors[evt.carrier].state binds carrier.
@precondition function precondition(evt::Mutate, physical)
    return physical.actors[evt.carrier].state == Infectious
end

function enable(evt::Mutate, physical, when)
    mutate_rate = 0.01
    return (Exponential(inv(mutate_rate)), when)
end

@fire function fire!(evt::Mutate, physical, when, rng)
    strain_idx = physical.actors[evt.carrier].strain
    infectivity = log(physical.strains[strain_idx].infectivity)
    virulence = log(physical.strains[strain_idx].virulence)
    sigma = PDiagMat([0.1, 0.1])
    child_rates = exp.(rand(rng, MvNormal([infectivity, virulence], sigma)))
    child = Strain(child_rates[1], child_rates[2], strain_idx)
    child_id = physical.next_strain_id
    physical.strains[child_id] = child
    physical.next_strain_id += 1
    physical.actors[evt.carrier].strain = child_id
end

struct InitEvent <: SimEvent end

@fire function fire!(evt::InitEvent, physical, when, rng)
    for pidx in eachindex(physical.actors)
        haunt_idx = rand(rng, 1:length(physical.actor_params[pidx].haunts))
        physical.actors[pidx].haunt = haunt_idx
        loc_idx = physical.actor_params[pidx].haunts[haunt_idx]
        push!(physical.locations[loc_idx].individuals, pidx)
        physical.locations[loc_idx].individual_cnt += 1
    end

    @assert length(physical.strains) == 1
    infect_cnt = round(Int, 0.1 * length(physical.actors))
    for pidx in 1:infect_cnt
        physical.actors[pidx].strain = 1
        physical.actors[pidx].state = Infectious
    end

    for sidx in 2:min(infect_cnt, 5)
        fire!(Mutate(sidx), physical, when, rng)
    end
    @info "Infected $infect_cnt with $(length(physical.strains)) strains."
end

function run_sirvillage_derived()
    person_cnt = 30
    location_cnt = 10
    day_length = 1.0
    days = 30.0 * day_length
    rng = Xoshiro(2938423)
    physical = Village(person_cnt, location_cnt, day_length, rng)
    included_transitions = [InitEvent, Travel, Infect, Recover, Reset, Mutate]
    sim = SimulationFSM(
        physical, included_transitions; rng=rng
    )
    stop_condition = function (physical, step_idx, event, when)
        return when > days
    end
    ChronoSim.run(sim, InitEvent(), stop_condition)
    return sim.when
end

end
