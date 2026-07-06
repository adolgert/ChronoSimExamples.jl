# Phase 1c end-to-end on the example models: deterministic `replay` reproduces
# recorded runs of the derived elevator and sirvillage twins, and
# `guard_clauses` agrees clause-by-clause with the real `precondition` at
# replayed intermediate states. The sirvillage case also exercises a >=1000-step
# replay (the perf comparison is recorded manually in the PR, not asserted here).

using ChronoSim
using ChronoSim: ProbePolicy
using CompetingClocks
using Random
using Test
using Logging

using ChronoSimExamples: ElevatorDerivedExample, ElevatorExample, SIRVillageDerived
const ED = ElevatorDerivedExample
const SV = SIRVillageDerived

########## Elevator (derived twin) ##########

const _ELEV_EVENTS = [
    ED.PickNewDestination, ED.CallElevator, ED.OpenElevatorDoors, ED.EnterElevator,
    ED.ExitElevator, ED.CloseElevatorDoors, ED.MoveElevator, ED.StopElevator,
    ED.DispatchElevator,
]

function _record_elevator(; person=1, elevator=1, floors=3, minutes=120.0, seed=93472934)
    physical = ED.ElevatorSystem(person, elevator, floors)
    rec = RecordSkeleton(; metadata=(model="elevator", seed=seed))
    sim = SimulationFSM(physical, _ELEV_EVENTS;
        sampler=NextReactionMethod(), key_type=Tuple, rng=Xoshiro(seed), policy=rec)
    stop = (p, i, e, w) -> w > minutes
    ChronoSim.run(sim, ED.init_physical, stop)
    return recorded_skeleton(rec)
end

function _elevator_factory(policy; person=1, elevator=1, floors=3, seed=93472934)
    physical = ED.ElevatorSystem(person, elevator, floors)
    sim = SimulationFSM(physical, _ELEV_EVENTS;
        sampler=NextReactionMethod(), key_type=Tuple, rng=Xoshiro(seed), policy=policy)
    return (sim, ED.init_physical)
end

# Every registered elevator event over the small enumerated domains.
function _all_elevator_events(phys)
    evts = SimEvent[]
    for p in 1:length(phys.person)
        push!(evts, ED.PickNewDestination(p), ED.CallElevator(p))
    end
    for e in 1:length(phys.elevator)
        push!(evts, ED.OpenElevatorDoors(e), ED.EnterElevator(e), ED.ExitElevator(e),
            ED.CloseElevatorDoors(e), ED.MoveElevator(e), ED.StopElevator(e))
    end
    for f in 1:phys.floor_cnt, d in (ED.Up, ED.Down)
        push!(evts, ED.DispatchElevator(f, d))
    end
    return evts
end

# Reconstruct the real precondition's verdict from clause values: the first
# non-`true` value decides. Returns (:accept,) / (:reject,) / (:throws, ex).
function _predict(clauses)
    for (_, v) in clauses
        v === true && continue
        v === false && return (:reject, nothing)
        return (:throws, v)          # an exception before any false
    end
    return (:accept, nothing)
end

@testset "elevator replay reproduces exactly" begin
    skel = _record_elevator()
    @test length(skel.steps) > 0
    sim = replay(_elevator_factory, skel)          # no ReplayDivergence
    @test sim.when == skel.steps[end].when
end

@testset "elevator guard_clauses agrees with precondition" begin
    skel = _record_elevator()
    n = length(skel.steps)
    for k in unique([0, min(25, n), min(50, n)])
        sim = replay(_elevator_factory, skel; upto=k)
        phys = sim.physical
        for evt in _all_elevator_events(phys)
            clauses = guard_clauses(evt, phys)
            kind, payload = _predict(clauses)
            if kind === :throws
                @test_throws typeof(payload) precondition(evt, phys)
            else
                @test precondition(evt, phys) == (kind === :accept)
            end
        end
    end
end

@testset "hand-written @guard events are guard_clauses-covered" begin
    # Phase 3: the hand-written elevator preconditions are annotated @guard, which
    # emits the precondition verbatim (byte-identical runtime) AND bakes
    # precondition_ast — so guard_clauses now evaluates them. Previously a
    # @conditionsfor-only event baked no precondition_ast and guard_clauses refused
    # it with :no_precondition; @guard closes that registry gap for hand-written
    # models. A freshly built hand system exercises the now-live clause analysis.
    system = ElevatorExample.ElevatorSystem(1, 1, 3)
    clauses = guard_clauses(ElevatorExample.OpenElevatorDoors(1), system)
    @test clauses isa AbstractVector
    @test !isempty(clauses)
end

@testset "whynot ingredient: rejected conjunct is named" begin
    # A freshly built system (no init): elevator stationary, no calls requested,
    # no buttons pressed. OpenElevatorDoors(1) fails on `call_exists || button_pressed`.
    system = ED.ElevatorSystem(1, 1, 3)
    clauses = guard_clauses(ED.OpenElevatorDoors(1), system)
    i = findfirst(c -> occursin("call_exists", c[1]), clauses)
    @test i !== nothing
    @test clauses[i][2] === false
    @test precondition(ED.OpenElevatorDoors(1), system) == false
end

########## SIRVillage (derived twin) ##########

const _SIRV_EVENTS = [SV.InitEvent, SV.Travel, SV.Infect, SV.Recover, SV.Reset, SV.Mutate]

function _record_sirvillage(; N=30, L=10, days=60.0, seed=2938423)
    rng = Xoshiro(seed)
    physical = SV.Village(N, L, 1.0, rng)
    rec = RecordSkeleton(; metadata=(model="sirvillage", N=N, seed=seed))
    sim = SimulationFSM(physical, _SIRV_EVENTS; rng=rng, policy=rec)
    stop = (p, i, e, w) -> w > days
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, SV.InitEvent(), stop)
    end
    return recorded_skeleton(rec)
end

function _sirvillage_factory(policy; N=30, L=10, seed=2938423)
    rng = Xoshiro(seed)
    physical = SV.Village(N, L, 1.0, rng)
    sim = SimulationFSM(physical, _SIRV_EVENTS; rng=rng, policy=policy)
    return (sim, SV.InitEvent())
end

@testset "sirvillage 1000-step replay" begin
    skel = _record_sirvillage(; days=60.0)
    @test length(skel.steps) >= 1000
    sim = with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        replay(_sirvillage_factory, skel)          # no ReplayDivergence
    end
    @test sim.when == skel.steps[end].when
end
