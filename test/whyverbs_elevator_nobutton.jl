# TEST FIXTURE for Phase 1e whynot acceptance scenario 2 (docs/design/phase1e_design.md).
#
# This is a MAINTAINED COPY of src/elevator/elevator_derived.jl (the @precondition
# twin, so guard_clauses clause analysis is available) with ONE line deleted from
# EnterElevator's fire!: the `elevator.buttons_pressed = union(...)` that presses
# the boarding person's destination button. Without it a person boards but presses
# no button, so an elevator that was never dispatched has no call and no button to
# open its doors — OpenElevatorDoors on that instance stays PROPOSED BUT REJECTED
# for the whole run, which `whynot` must diagnose. Keep this copy in sync with the
# shipping derived module except for that one deletion.
#
# Derived-generator twin of ElevatorExample (src/elevator/elevator.jl).
#
# All 9 events have their generators derived from the precondition body via
# `@precondition`. Features per derived event:
#   PickNewDestination, CallElevator, OpenElevatorDoors, ExitElevator:
#       plain precondition reads (clean and loop-widened triggers).
#   EnterElevator: calls the `people_waiting` @fragment helper (the feature
#       showcase). The helper receives state and is inlined for analysis; its loop
#       over eachindex(person) widens the person reads, and get_direction (also
#       @fragment) inlines inside it.
#   CloseElevatorDoors: precondition-recursion — the body calls
#       precondition(EnterElevator(evt.elevator_idx), system) and
#       precondition(ExitElevator(evt.elevator_idx), system); each called
#       precondition's registered body is inlined with the constructor arg substituted
#       for its event field (a CLEAN elevator_idx binding).
#   DispatchElevator: any(... for elevator in system.elevator) reducers over a state
#       container (analyzed like loops); the second reducer calls the get_direction
#       @fragment helper on elevator state.
#
#   MoveElevator: dict tuple-key iteration (widened) + can_service_call @fragment +
#       a zero-binding trigger on the top-level scalar floor_cnt.
#   StopElevator: precondition-recursion into OpenElevatorDoors + the floor_cnt
#       scalar trigger.
#
# All nine events derive. (An earlier revision kept MoveElevator/StopElevator manual
# because a top-level scalar-field read crashed the derivation; ChronoSim now derives
# scalar reads as zero-binding triggers.)
#
# Helpers get_direction, can_service_call, and people_waiting are marked @fragment so
# a precondition may pass state into them (an unmarked helper receiving state is
# out-of-fragment). Event definition order is unchanged from the hand-written twin:
# every precondition-recursion target (OpenElevatorDoors, EnterElevator, ExitElevator)
# already precedes its caller (StopElevator, CloseElevatorDoors), which the derivation
# requires (a called precondition's body must be registered first), so no reordering
# was needed.
#
# Distinct struct types (parallel to ElevatorExample) are required. clock_key uses
# nameof(type), so derived and hand-written events with the same name share clock
# keys and their trajectories compare directly. TLA recorder parts are not ported.
module ElevatorNoButton
using CompetingClocks
using Distributions
using Logging
using Random
using ChronoSim
using ChronoSim.ObservedState
import ChronoSim: precondition, enable, fire!, generators

@enum ElevatorDirection Up Down Stationary
# CompetingClocks 0.4 seeds each clock's random stream from `hash((seed, clock_key))`,
# and DispatchElevator carries this enum inside its clock key. An enum has no
# content-based `Base.hash`, so it falls back to `objectid`, which differs between
# the twin modules that define the "same" enum and is not even stable across julia
# processes with different compile options -- both reproducibility and the
# hand-vs-derived trajectory identity break without this method. Hashing by the
# value's name makes the clock key hash content-based and module-independent.
Base.hash(x::ElevatorDirection, h::UInt) = hash(Symbol(x), h)

@keyedby Person Int64 begin
    location::Int64
    destination::Int64
    elevator::Int64
    waiting::Bool
end

@keyedby ElevatorCall Tuple{Int64,ElevatorDirection} begin
    requested::Bool
end

@keyedby Elevator Int64 begin
    floor::Int64
    direction::ElevatorDirection
    doors_open::Bool
    buttons_pressed::Set{Int64}
end

@observedphysical ElevatorSystem begin
    person::ObservedVector{Person,Member}
    calls::ObservedDict{Tuple{Int64,ElevatorDirection},ElevatorCall,Member}
    elevator::ObservedVector{Elevator,Member}
    floor_cnt::Int64
end

function ElevatorSystem(person_cnt::Int64, elevator_cnt::Int64, floor_cnt::Int64)
    persons = ObservedArray{Person,Member}(undef, person_cnt)
    for pidx in eachindex(persons)
        persons[pidx] = Person(1, 1, 0, false)
    end
    calls = ObservedDict{Tuple{Int64,ElevatorDirection},ElevatorCall,Member}()
    for flooridx in 1:floor_cnt
        for direction in [Up, Down]
            calls[(flooridx, direction)] = ElevatorCall(false)
        end
    end
    elevators = ObservedArray{Elevator,Member}(undef, elevator_cnt)
    for elevidx in eachindex(elevators)
        elevators[elevidx] = Elevator(1, Stationary, false, Set{Int64}())
    end
    ElevatorSystem(persons, calls, elevators, floor_cnt)
end

######## Helper functions

get_distance(floor1, floor2) = abs(floor1 - floor2)

# @fragment so preconditions may pass elevator/person state into these helpers; the
# analysis inlines the registered body (runtime still calls the function unchanged).
# get_direction must be registered before people_waiting, which calls it.
@fragment get_direction(current, destination) = destination > current ? Up : Down
@fragment function can_service_call(elevator, call_floor, call_dirn)
    elevator.floor == call_floor && elevator.direction == call_dirn
end

@fragment function people_waiting(people, floor, dirn)
    waiters = Int[]
    for pidx in eachindex(people)
        p = people[pidx]
        if p.location == floor && p.waiting && get_direction(p.location, p.destination) == dirn
            push!(waiters, pidx)
        end
    end
    return waiters
end

################# Events follow

struct PickNewDestination <: SimEvent
    person::Int64
end

# Derived: reads person[evt.person].waiting and .location -> two clean triggers
# binding person (hand-written reacted only to .location; the extra .waiting
# trigger proposes more candidates that the precondition filters).
@precondition function precondition(evt::PickNewDestination, system)
    person = system.person[evt.person]
    return !person.waiting && person.location != 0
end

enable(evt::PickNewDestination, system, when) = (Exponential(1.0), when)

function fire!(evt::PickNewDestination, system, when, rng)
    who = system.person[evt.person]
    dests = Set(collect(1:system.floor_cnt))
    delete!(dests, system.person[evt.person].location)
    system.person[evt.person].destination = rand(rng, dests)
end

struct CallElevator <: SimEvent
    person::Int64
end

# The location != 0 guard makes the precondition self-contained: without it the
# event is enabled for a person inside an elevator and fire! crashes on
# calls[(0, dir)]. The hand-written model only avoided that state by trigger
# narrowness (reacting to destination alone), which is exactly the
# non-self-contained pattern derivation exposes; the hand-written twin adopted
# the same guard, which provably does not change its trajectories.
@precondition function precondition(evt::CallElevator, system)
    person = system.person[evt.person]
    return person.location != 0 && person.location != person.destination && !person.waiting
end

enable(evt::CallElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::CallElevator, system, when, rng)
    person = system.person[evt.person]
    person.waiting = true
    direction = get_direction(person.location, person.destination)
    any_open = any(
        can_service_call(system.elevator[elidx], person.location, direction) &&
        system.elevator[elidx].doors_open for elidx in eachindex(system.elevator)
    )
    if !any_open
        system.calls[(person.location, direction)].requested = true
    end
    @assert system.person[evt.person].waiting
end

struct OpenElevatorDoors <: SimEvent
    elevator_idx::Int64
end

# Derived: clean triggers on elevator.direction/floor/buttons_pressed/doors_open
# (bind elevator_idx) plus a widened trigger on calls.requested (the tuple key
# (elevator.floor, elevator.direction) is state-derived). elevator_idx is inferred
# as eachindex(physical.elevator) from the clean reads, so no @domain is needed.
# This subsumes the hand-written triggers, including the doors_open one added by
# the Phase 2 finding.
@precondition function precondition(evt::OpenElevatorDoors, system)
    elevator = system.elevator[evt.elevator_idx]
    call_exists =
        elevator.direction != Stationary &&
        system.calls[(elevator.floor, elevator.direction)].requested
    button_pressed = elevator.floor ∈ elevator.buttons_pressed
    return !elevator.doors_open && (call_exists || button_pressed)
end

enable(evt::OpenElevatorDoors, system, when) = (Exponential(1.0), when)

function fire!(evt::OpenElevatorDoors, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]
    elevator.doors_open = true

    if elevator.floor ∈ elevator.buttons_pressed
        elevator.buttons_pressed = setdiff(elevator.buttons_pressed, elevator.floor)
    end
    system.calls[(elevator.floor, elevator.direction)].requested = false
end

struct EnterElevator <: SimEvent
    elevator_idx::Int64
end

# Derived, feature showcase: the precondition CALLS the people_waiting @fragment
# helper, written once and shared with fire!. The helper is inlined for analysis;
# its loop over eachindex(system.person) taints the person reads into widened
# triggers (person.location/waiting/destination — the last via the inlined
# get_direction), while elevator.doors_open/direction bind elevator_idx cleanly.
# elevator_idx is inferred from the clean reads; no @domain. Compared to the
# hand-written module, this adds triggers on elevator.direction and
# person.location/destination (extra candidates the precondition filters).
@precondition function precondition(evt::EnterElevator, system)
    elevator = system.elevator[evt.elevator_idx]
    elevator_ready = (elevator.doors_open && elevator.direction != Stationary)
    people_ready = !isempty(people_waiting(system.person, elevator.floor, elevator.direction))
    return elevator_ready && people_ready
end

enable(evt::EnterElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::EnterElevator, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]

    entered_cnt = 0
    for pidx in 1:length(system.person)
        person = system.person[pidx]
        if person.location == elevator.floor && person.waiting
            person_direction = person.destination > person.location ? Up : Down
            if elevator.direction == Stationary || elevator.direction == person_direction
                person.location = 0
                person.elevator = evt.elevator_idx
                person.waiting = false
                # DELETED for scenario 2: the boarding person presses no destination
                # button, so a never-dispatched elevator never gets a reason to open.
                # elevator.buttons_pressed = union(elevator.buttons_pressed, person.destination)
                entered_cnt += 1
            end
        end
    end
    @assert entered_cnt > 0
end

struct ExitElevator <: SimEvent
    elevator_idx::Int64
end

# Derived: widened triggers on person.elevator/destination (loop-indexed) plus
# clean triggers on elevator.floor/doors_open. elevator_idx inferred; no @domain.
@precondition function precondition(evt::ExitElevator, system)
    elevator = system.elevator[evt.elevator_idx]

    anyone_exit_here = false
    for pidx in 1:length(system.person)
        person = system.person[pidx]
        if person.elevator == evt.elevator_idx && person.destination == elevator.floor
            anyone_exit_here = true
        end
    end

    return anyone_exit_here && elevator.doors_open
end

enable(evt::ExitElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::ExitElevator, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]

    for pidx in 1:length(system.person)
        person = system.person[pidx]
        if person.elevator == evt.elevator_idx && person.destination == elevator.floor
            person.location = elevator.floor
            person.elevator = 0
            person.waiting = false
        end
    end
end

struct CloseElevatorDoors <: SimEvent
    elevator_idx::Int64
end

# Derived via precondition-recursion: the body calls precondition(EnterElevator(...))
# and precondition(ExitElevator(...)); both are @precondition-defined earlier, so their
# registered bodies inline here with evt.elevator_idx substituted (clean elevator_idx
# bindings). The two explicit loops widen the person reads. The fired(EnterElevator)/
# fired(ExitElevator) triggers of the hand-written module are subsumed: the derived
# triggers react to the person/elevator state those events write.
@precondition function precondition(evt::CloseElevatorDoors, system)
    elevator = system.elevator[evt.elevator_idx]

    person_needs_to_board = false
    for pidx in 1:length(system.person)
        person = system.person[pidx]
        if person.location == elevator.floor && person.waiting
            person_direction = person.destination > person.location ? Up : Down
            if elevator.direction == Stationary || elevator.direction == person_direction
                person_needs_to_board = true
            end
        end
    end

    person_needs_to_exit = false
    for pidx in 1:length(system.person)
        person = system.person[pidx]
        if person.elevator == evt.elevator_idx && person.destination == elevator.floor
            person_needs_to_exit = true
        end
    end

    enabled_enter = precondition(EnterElevator(evt.elevator_idx), system)
    enabled_exit = precondition(ExitElevator(evt.elevator_idx), system)

    return elevator.doors_open &&
           !(person_needs_to_board || person_needs_to_exit) &&
           !enabled_enter &&
           !enabled_exit
end

enable(evt::CloseElevatorDoors, system, when) = (Exponential(1.0), when)

function fire!(evt::CloseElevatorDoors, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]
    elevator.doors_open = false
end

struct MoveElevator <: SimEvent
    elevator_idx::Int64
end

# Derived. The system.floor_cnt bounds check reads a top-level scalar field, which
# derives a zero-binding trigger on (floor_cnt,); the dict tuple-key iteration widens
# onto calls, and can_service_call is an @fragment helper.
@precondition function precondition(evt::MoveElevator, system)
    elevator = system.elevator[evt.elevator_idx]
    next_floor = elevator.direction == Up ? elevator.floor + 1 : elevator.floor - 1
    next_floor_valid = next_floor >= 1 && next_floor <= system.floor_cnt
    stop_here = elevator.floor ∈ elevator.buttons_pressed
    other_calls_serviced = true
    for ((floor, direction), call) in system.calls
        call.requested || continue
        if can_service_call(elevator, floor, direction)
            another_can_service = false
            for other_elev in eachindex(system.elevator)
                other_elev == evt.elevator_idx && continue
                other_elevator = system.elevator[other_elev]
                another_can_service |= can_service_call(other_elevator, floor, direction)
            end
            other_calls_serviced &= another_can_service
        end
    end
    return !elevator.doors_open &&
           elevator.direction != Stationary &&
           next_floor_valid &&
           !stop_here &&
           other_calls_serviced
end

enable(evt::MoveElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::MoveElevator, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]
    elevator.floor += elevator.direction == Up ? 1 : -1
end

struct StopElevator <: SimEvent
    elevator_idx::Int64
end

# Derived. The floor_cnt scalar read derives a zero-binding trigger, and the
# doors_will_open check inlines OpenElevatorDoors' registered precondition with
# evt.elevator_idx substituted, keeping its reads cleanly bound.
@precondition function precondition(evt::StopElevator, system)
    elevator = system.elevator[evt.elevator_idx]
    next_floor = elevator.direction == Up ? elevator.floor + 1 : elevator.floor - 1
    next_floor_valid = 1 <= next_floor <= system.floor_cnt
    doors_will_open = precondition(OpenElevatorDoors(evt.elevator_idx), system)
    return !elevator.doors_open && !next_floor_valid && !doors_will_open
end

enable(evt::StopElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::StopElevator, system, when, rng)
    elevator = system.elevator[evt.elevator_idx]
    elevator.direction = Stationary
end

struct DispatchElevator <: SimEvent
    floor::Int64
    direction::ElevatorDirection
end

# Derived: the any(... for elevator in system.elevator) reducers are analyzed like
# loops — the element alias `elevator` taints elevator.direction/floor into widened
# triggers, and the second reducer's get_direction(elevator.floor, evt.floor) inlines
# (@fragment) to a clean read of the same elevator.floor. The clean tuple-key read
# calls[(evt.floor, evt.direction)].requested binds both event fields; floor and
# direction resolve their domains from the calls dict's tuple key, so no @domain.
@precondition function precondition(evt::DispatchElevator, system)
    call_active = system.calls[(evt.floor, evt.direction)].requested
    any_stationary = any(elevator.direction == Stationary for elevator in system.elevator)
    any_approaching = any(
        elevator.direction == evt.direction &&
        (elevator.floor == evt.floor || get_direction(elevator.floor, evt.floor) == evt.direction)
        for elevator in system.elevator
    )
    return call_active && (any_stationary || any_approaching)
end

enable(evt::DispatchElevator, system, when) = (Exponential(1.0), when)

function fire!(evt::DispatchElevator, system, when, rng)
    close_elev = 0
    close_dist = system.floor_cnt + 1
    for elev_idx in eachindex(system.elevator)
        elevator = system.elevator[elev_idx]
        approaching =
            elevator.direction == evt.direction && (
                elevator.floor == evt.floor ||
                get_direction(elevator.floor, evt.floor) == evt.direction
            )
        if elevator.direction == Stationary || approaching
            dist = get_distance(elevator.floor, evt.floor)
            if dist < close_dist
                close_elev = elev_idx
                close_dist = dist
            end
        end
    end
    @assert close_elev > 0
    elevator = system.elevator[close_elev]
    if elevator.direction == Stationary
        elevator.direction = evt.direction
        elevator.floor = evt.floor
    end
end

function init_physical(physical, when, rng)
    for pidx in eachindex(physical.person)
        physical.person[pidx].location = rand(rng, 1:physical.floor_cnt)
        physical.person[pidx].destination = physical.person[pidx].location
        physical.person[pidx].elevator = 0
        physical.person[pidx].waiting = false
    end
end

function run_elevator_derived()
    person_cnt = 1
    elevator_cnt = 1
    floor_cnt = 3
    minutes = 120.0
    physical = ElevatorSystem(person_cnt, elevator_cnt, floor_cnt)
    included_transitions = [
        PickNewDestination,
        CallElevator,
        OpenElevatorDoors,
        EnterElevator,
        ExitElevator,
        CloseElevatorDoors,
        MoveElevator,
        StopElevator,
        DispatchElevator,
    ]
    @assert length(included_transitions) == 9
    sim = SimulationFSM(
        physical, included_transitions; sampler=NextReactionMethod(), key_type=Tuple,
        rng=Xoshiro(93472934)
    )
    stop_condition = function (physical, step_idx, event, when)
        return when > minutes
    end
    ChronoSim.run(sim, init_physical, stop_condition)
    return sim.when
end

end
