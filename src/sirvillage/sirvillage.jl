module SIRVillage
using ChronoSim
using ChronoSim.ObservedState
using CompetingClocks
import ChronoSim: generators, precondition, enable, reenable, fire!
using Distributions
using Random
using PDMats

export run_sirvillage

"""
The goal is to make a disease simulation that's a little bit fun. You may
have seen disease simulations that show susceptible-infectious-recovered (SIR)
transitions for individuals. That's easy to do without any framework, so let's
make a halfway realistic simulation that has:

 1. The rate of recovery from disease is Gamma-distributed, which is the classic
    rate use in disease studies, not Exponential.

 1. Each individual has a different susceptibility to infection. Epidemiologists
    have long understood this is crucial for understanding disease spread.

 1. These people move around a city, or let's say a village, so that they
    have localized sets of contacts. Each person follows a semi-Markovian walk
    around a subset of locations.

 1. The evolutionary ecologist, Cyd Hamilton, railed at me for not including
    evolution in SIR simulations, so let's make the disease, itself, mutate into
    strains that have different infectivity and virulence. This raises more
    questions. Let's decide that every new strain escapes the previous one,
    so it can reinfect a person, but that people can't be infected by any
    strain while in infectious and recovered states.

There are a lot of choices, and a lot of parameters in such a system. We will
try to be conservative.
"""

@enum DiseaseState Susceptible Infectious Recovered

# Using keyedby says the fields of this struct can be used to track events.
@keyedby Individual Int64 begin
    state::DiseaseState
    strain::Int64
    haunt::Int64  # Index to their location within the IndividualParams.haunts vector.
end

"""
Characterize an individual with

 * A base level of resistance to infection and speed of recovery.
 * A subset of the village in which to move.
 * We will use a movement model that is parametrized on how long they spend
   at each place. The values here are derived from a dwell time and a time-spent
   fraction.
"""
struct IndividualParams
    robustness::Float64
    haunts::Vector{Int64}      # Subset of locations where this individual goes.
    exit_sum::Vector{Float64}  # Average time individual spends at each location.
    exit_frac::Vector{Float64} # Fraction of exits to a particular location.
end

@keyedby Location Int64 begin
    individual_cnt::Int64
    individuals::Vector{Int64}  # Repeats state from Individual.location.
end

@keyedby Strain Int64 begin
    infectivity::Float64  # Affects rate of infection.
    virulence::Float64    # Affects rate of recovery.
    parent::Int64
end


# This is the physical state of the system. It will report its reads and writes.
@observedphysical Village begin
    actors::ObservedVector{Individual}
    actor_params::Vector{IndividualParams}
    locations::ObservedVector{Location}
    strains::ObservedVector{Strain}
end


"""
This movement model is described in the docs as `dwell.md`.
For a useful simulation, there is a lot more work before the simulation starts
to create a world where the mixing of individuals reflects something useful,
and there is lots of data about human movement. This function shows a
parametrization for a movement model that might be helpful instead of thinking
about transition rates in a Markov model.
"""
function movement_model(location_cnt, individual_location_cnt, day_length, rng)
    dwells = 0.1 * rand(rng, individual_location_cnt)
    if individual_location_cnt > 2
        dwells[1] = 0.4 * day_length  # Home
        dwells[2] = 0.3 * day_length  # Work
    end
    fractions = rand(rng, individual_location_cnt)
    fractions /= sum(fractions)
    exit_sum = @. (2 / π) * dwells^2
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
    actors = ObservedArray{Individual}(undef, actor_cnt)
    for i in 1:actor_cnt
        actors[i] = Individual(Susceptible, 0, 0)
    end
    locations = ObservedArray{Location}(undef, location_cnt)
    for loc_idx in 1:location_cnt
        locations[loc_idx] = Location(0, Int64[])
    end
    strains = ObservedArray{Strain}(undef, 1)
    initial_infectivity = 1.0
    initial_virulence = 1.0
    strains[1] = Strain(initial_infectivity, initial_virulence, 0)
    return Village(actors, params, locations, strains)
end


"""
The Travel event fires when an individual will leave a location. When it fires
it chooses which location is the destination. We could alternatively create
a Travel event for every future locaiton of an individual, which would make
sense if we wanted to use a more complex semi-Markov structure that used
a different distribution of travel times for different destinations.
"""
struct Travel <: SimEvent
    who::Int64
end

precondition(event::Travel, physical) = true

@conditionsfor Travel begin
    @reactto changed(actors[who].haunt) do physical
        generate(Travel(who))
    end
end

function enable(evt::Travel, physical, when)
    # Julia's Weibull has the form exp(-(x/scale)^alpha).
    # Ours is exp(-(A/2) x^2), so A/2 = 1/scale^2, or scale = sqrt(2/A)
    haunt_idx = physical.actors[evt.who].haunt
    scale = 2.0 / physical.actor_params[evt.who].exit_sum[haunt_idx]
    return (Weibull(2, scale), when)
end

function fire!(evt::Travel, physical, when, rng)
    source_haunt = physical.actors[evt.who].haunt
    source_loc = physical.actor_params[evt.who].haunts[source_haunt]
    # Use a mask to eliminate travel to the place you started.
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

function precondition(evt::Infect, physical)
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

@conditionsfor Infect begin
    @reactto changed(actors[who].state) do physical
        loc_idx = physical.actor_params[who].haunts[physical.actors[who].haunt]
        # Just became infectious, so infect everybody at your location.
        if physical.actors[who].state == Infectious
            for sink in physical.locations[loc_idx].individuals
                generate(Infect(who, sink))
            end
        # Just became susceptible, so anybody can infect you.
        elseif physical.actors[who].state == Susceptible
            for source in physical.locations[loc_idx].individuals
                generate(Infect(source, who))
            end
        end
    end
end

function enable(evt::Infect, physical, when)
    strain_idx = physical.actors[evt.source].strain
    infectivity = physical.strains[strain_idx].infectivity
    robust = physical.actor_params[evt.sink].robustness
    # Julia exponential uses a scale parameter.
    return (Exponential(inv(infectivity / robust)), when)
end

function fire!(evt::Infect, physical, when, rng)
    strain_idx = physical.actors[evt.source].strain
    physical.actors[evt.sink].strain = strain_idx
    physical.actors[evt.sink].state = Infectious
end


struct Recover <: SimEvent
    who::Int64
end

precondition(evt::Recover, physical) = physical.actors[evt.who].state == Infectious

@conditionsfor Recover begin
    @reactto changed(actors[who].state) do physical
        generate(Recover(who))
    end
end

function enable(evt::Recover, physical, when)
    strain_idx = physical.actors[evt.who].strain
    virulence = physical.strains[strain_idx].virulence
    robust = physical.actors[evt.who].robustness
    # The more virulent, the longer the recovery.
    return (Gamma(3, inv(robust / virulence)), when)
end

function fire!(evt::Recover, physical, when, rng)
    physical.actors[evt.who].state = Recovered
end


struct Reset <: SimEvent
    who::Int64
end

precondition(evt::Reset, physical) = physical.actors[evt.who].state == Recovered

@conditionsfor Reset begin
    @reactto changed(actors[who].state) do physical
        generate(Reset(who))
    end
end

function enable(evt::Reset, physical, when)
    return (Exponential(inv(0.05)), when)
end

function fire!(evt::Reset, physical, when, rng)
    physical.actors[evt.who].state = Susceptible
    physical.actors[evt.who].strain = 0
end


"""
Mutation of strains is associated with people who are sick. The more people
infected, the more likely is mutation. Mutation happens within the sick person.
"""
struct Mutate <: SimEvent
    carrier::Int64
end

function precondition(evt::Mutate, physical)
    return physical.actors[evt.carrier].state == Infectious
end

@conditionsfor Mutate begin
    @reactto changed(actors[who].state) do physical
        generate(Mutate(who))
    end
end

function enable(evt::Mutate, physical, when)
    mutate_rate = 0.01
    return (Exponential(inv(mutate_rate)), when)
end

function fire!(evt::Mutate, physical, when, rng)
    strain_idx = physical.actors[evt.carrier].strain
    # Use log space to ensure rates remain positive.
    infectivity = log(physical.strains[strain_idx].infectivity)
    virulence = log(physical.strains[strain_idx].virulence)
    # I have no idea if these numbers will work well for a simulation.
    sigma = PDiagMat([0.1, 0.1])
    # Use a multivariate normal to constrain evolution to either
    # infectivity or virulence, on average.
    child_rates = exp.(rand(rng, MvNormal([infectivity, virulence], sigma)))
    child = Strain(child_rates[1], child_rates[2], strain_idx)
    push!(physical.strains, child)
    physical.actors[evt.carrier].strain = length(physical.strains)
end


function init_physical!(physical, when, rng)
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

    # Creates a few strains to start.
    for sidx in 2:min(infect_cnt, 5)
        fire!(Mutate(sidx), physical, when, rng)
    end
end

function validate_invariants(physical)
    err_msg = String[]
    strain_cnt = length(physical.strains)
    if strain_cnt < 1
        push!(err_msg, "There are no physical.strains")
    end
    for sidx in eachindex(physical.strains)
        strain = physical.strains[sidx]
        if strain.infectivity < 0.0 || strain.virulence < 0.0
            push!(
                err_msg,
                "Strain $sidx infectivity=$(strain.infectivity) and virulence $(strain.virulence)"
                )
        end
        if strain.parent < 0 || strain.parent > length(physical.strains)
            push!(err_msg, "Strain $sidx parent $(strain.parent)")
        end
    end
    for loc_idx in eachindex(physical.locations)
        cnt = physical.locations[loc_idx].individual_cnt
        real_len = length(physical.locations[loc_idx].individuals)
        if real_len != cnt
            push!(err_msg, "Location $loc_idx has wrong individuals $cnt not $real_len")
        end
    end
    for act_idx in eachindex(physical.actors)
        actor = physical.actors[act_idx]
        if actor.state != Susceptible && actor.strain == 0
            push!(err_msg, "Actor $act_idx is sick with no strain.")
        end
    end
    return err_msg
end

struct TrajectoryEntry
    event::Tuple
    when::Float64
end

mutable struct TrajectorySave
    trajectory::Vector{TrajectoryEntry}
    sim::SimulationFSM
    TrajectorySave() = new(Vector{TrajectoryEntry}())
end

function (te::TrajectorySave)(physical, when, event, changed_places)
    @info "Firing $event at $when"
    @info "Enabled events $(keys(te.sim.enabled_events))"
    push!(te.trajectory, TrajectoryEntry(clock_key(event), when))
    err_str = vcat(validate_invariants(physical))
    if !isempty(err_str)
        error(join(err_str, "\n"))
    end
end

function run_sirvillage()
    person_cnt = 1
    location_cnt = 10
    day_length = 1.0
    days = 10000.0 * day_length
    Sampler = CombinedNextReaction{Tuple,Float64}
    rng = Xoshiro(2938423)
    physical = Village(person_cnt, location_cnt, day_length, rng)
    included_transitions = [
        Travel,
        Infect,
        Recover,
        Reset,
        Mutate,
    ]
    trajectory = TrajectorySave()
    sim = SimulationFSM(
        physical, Sampler(), included_transitions; rng=rng, observer=trajectory
    )
    trajectory.sim = sim
    # Stop-condition is called after the next event is chosen but before the
    # next event is fired. This way you can stop at an end time between events.
    stop_condition = function (physical, step_idx, event, when)
        return when > days
    end
    ChronoSim.run(sim, init_physical!, stop_condition)
    println("Simulation ended at $(sim.when) minutes.")
    return sim.when
end


end

if abspath(PROGRAM_FILE) == @__FILE__
    using .SIRVillage
    run_sirvillage()
end
