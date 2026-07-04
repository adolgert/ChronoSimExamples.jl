# Derived-generator twin of ElevatorExample (src/elevator/elevator.jl).
#
# Four events have their generators derived from the precondition body via
# `@precondition` (the hand-written `@conditionsfor` blocks are deleted):
#   PickNewDestination, OpenElevatorDoors, EnterElevator, ExitElevator
# Five events stay manual (each carries a `# stays manual:` note):
#   - CloseElevatorDoors, StopElevator: preconditions recurse into precondition(...)
#   - MoveElevator: dict tuple-destructure iteration + can_service_call helper
#   - DispatchElevator: any(...) over a state container
#
# EnterElevator inlines the people_waiting logic into the precondition as an
# explicit fixed-extent for-loop. The people_waiting helper itself receives state,
# so passing it (or any(...) over a state generator) to the precondition is out of
# fragment; the for-loop taints the person index into a widened trigger, which is
# in fragment and semantically identical.
#
# Distinct struct types (parallel to ElevatorExample) are required. clock_key uses
# nameof(type), so derived and hand-written events with the same name share clock
# keys and their trajectories compare directly. TLA recorder parts are not ported.
module ElevatorDerivedExample
using CompetingClocks
using Distributions
using Logging
using Random
using ChronoSim
using ChronoSim.ObservedState
import ChronoSim: precondition, enable, fire!, generators

@enum ElevatorDirection Up Down Stationary

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
get_direction(current, destination) = destination > current ? Up : Down
function can_service_call(elevator, call_floor, call_dirn)
    elevator.floor == call_floor && elevator.direction == call_dirn
end

# Kept for fire! parity with the hand-written module; the derived EnterElevator
# precondition inlines this logic (a helper receiving state is out of fragment).
function people_waiting(people, floor, dirn)
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

# Derived: the people_waiting helper is inlined as an explicit for-loop over
# eachindex(system.person). The loop index taints the person reads into widened
# triggers (person.location/destination/waiting), and elevator.doors_open/direction
# bind elevator_idx cleanly. elevator_idx is inferred from the clean reads; no
# @domain. Compared to the hand-written module, this adds triggers on
# elevator.direction and person.location/destination (extra candidates the
# precondition filters).
@precondition function precondition(evt::EnterElevator, system)
    elevator = system.elevator[evt.elevator_idx]
    elevator_ready = (elevator.doors_open && elevator.direction != Stationary)
    people_ready = false
    for pidx in eachindex(system.person)
        person = system.person[pidx]
        person_direction = person.destination > person.location ? Up : Down
        if person.location == elevator.floor &&
           person.waiting &&
           person_direction == elevator.direction
            people_ready = true
        end
    end
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
                elevator.buttons_pressed = union(elevator.buttons_pressed, person.destination)
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

# stays manual: precondition recurses into precondition(EnterElevator(...), system)
# and precondition(ExitElevator(...), system), passing state to an opaque function.
@conditionsfor CloseElevatorDoors begin
    @reactto changed(elevator[elidx].doors_open) do system
        generate(CloseElevatorDoors(elidx))
    end
    @reactto fired(EnterElevator(elidx)) do system
        generate(CloseElevatorDoors(elidx))
    end
    @reactto fired(ExitElevator(elidx)) do system
        generate(CloseElevatorDoors(elidx))
    end
end

function precondition(evt::CloseElevatorDoors, system)
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

# stays manual: the precondition iterates system.calls with tuple-key
# destructuring and calls the can_service_call helper on state; both are out of the
# derivable fragment.
@conditionsfor MoveElevator begin
    @reactto changed(elevator[elidx].doors_open) do system
        generate(MoveElevator(elidx))
    end
    @reactto changed(elevator[elidx].direction) do system
        generate(MoveElevator(elidx))
    end
    @reactto changed(elevator[elidx].floor) do system
        generate(MoveElevator(elidx))
    end
    @reactto changed(calls[callkey].requested) do system
        for elidx in 1:length(system.elevator)
            generate(MoveElevator(elidx))
        end
    end
end

function precondition(evt::MoveElevator, system)
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

# stays manual: precondition recurses into precondition(OpenElevatorDoors(...),
# system), passing state to an opaque function.
@conditionsfor StopElevator begin
    @reactto changed(elevator[elidx].floor) do system
        generate(StopElevator(elidx))
    end
    @reactto changed(elevator[elidx].doors_open) do system
        generate(StopElevator(elidx))
    end
end

function precondition(evt::StopElevator, system)
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

# stays manual: precondition uses any(... for elevator in system.elevator), a
# generator over a state container passed to an opaque reducer, out of fragment.
@conditionsfor DispatchElevator begin
    @reactto changed(calls[callkey].requested) do system
        floor, direction = callkey
        generate(DispatchElevator(floor, direction))
    end
    @reactto changed(elevator[elidx].direction) do system
        for (call_key, call) in system.calls
            if call.requested
                floor, direction = call_key
                generate(DispatchElevator(floor, direction))
            end
        end
    end
end

function precondition(evt::DispatchElevator, system)
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
    Sampler = CombinedNextReaction{Tuple,Float64}
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
        physical, included_transitions; sampler=Sampler(), rng=Xoshiro(93472934)
    )
    stop_condition = function (physical, step_idx, event, when)
        return when > minutes
    end
    ChronoSim.run(sim, init_physical, stop_condition)
    return sim.when
end

end
