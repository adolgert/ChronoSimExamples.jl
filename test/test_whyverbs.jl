# Phase 1e acceptance scenarios — the why-verbs' exit criterion
# (debugging_and_verification_plan.md §1e task 6). Each scenario reproduces a
# known bug class and asserts the matching verb explains it in one readout.
#
# Fixtures:
#   whyverbs_elevator_stopbug.jl  — ElevatorStopBug: hand-written elevator with the
#       53c18d2 StopElevator trigger fix reverted (scenario 1).
#   whyverbs_elevator_nobutton.jl — ElevatorNoButton: derived elevator with the
#       button-press line of EnterElevator's fire! deleted (scenario 2).

using ChronoSim
using ChronoSim: InvariantViolation, CheckInvariants, PolicyStack, RecordSkeleton,
    recorded_skeleton, clock_key
using ChronoSim.ObservedState
using CompetingClocks
using CompetingClocks: CombinedNextReaction
using Distributions
using Logging
using Random
using Test

using ChronoSimExamples: ElevatorDerivedExample, ElevatorExample, SIRVillage

include("whyverbs_elevator_stopbug.jl")   # module ElevatorStopBug
include("whyverbs_elevator_nobutton.jl")  # module ElevatorNoButton

const _ED = ElevatorDerivedExample
const _SB = ElevatorStopBug
const _NB = ElevatorNoButton

_elev_events(M) = [M.PickNewDestination, M.CallElevator, M.OpenElevatorDoors,
    M.EnterElevator, M.ExitElevator, M.CloseElevatorDoors, M.MoveElevator,
    M.StopElevator, M.DispatchElevator]

########## Scenario 1 — StopElevator NEVER PROPOSED (the reverted 53c18d2 bug) ##########

@testset "whynot: StopElevator never proposed (missing trigger)" begin
    P, E, F, seed = 3, 2, 5, 93472934
    # (a) The unmodified derived twin: discover the first StopElevator step.
    recd = RecordSkeleton()
    simd = SimulationFSM(_ED.ElevatorSystem(P, E, F), _elev_events(_ED);
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(seed), policy=recd)
    ChronoSim.run(simd, _ED.init_physical, (p, i, e, w) -> w > 120.0)
    skeld = recorded_skeleton(recd)
    s = findfirst(st -> st.clock[1] == :StopElevator, skeld.steps)
    @test s !== nothing
    k = skeld.steps[s].clock[2]

    # (b) The buggy hand-written twin, same seed, stopped just before StopElevator
    # would fire: the prefix state matches the derived run, so StopElevator(k)'s
    # precondition holds there by construction, yet nothing ever proposed it.
    recb = RecordSkeleton()
    simb = SimulationFSM(_SB.ElevatorSystem(P, E, F), _elev_events(_SB);
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(seed), policy=recb)
    ChronoSim.run(simb, _SB.init_physical, (p, i, e, w) -> i >= s)
    skelb = recorded_skeleton(recb)
    factory = policy -> (SimulationFSM(_SB.ElevatorSystem(P, E, F), _elev_events(_SB);
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(seed), policy=policy),
        _SB.init_physical)

    rep = whynot(skelb, factory, _SB.StopElevator(k))
    @test rep.stage == :never_proposed
    @test rep.detail.precondition_now === true
    # names an address from the true trigger set that the buggy trigger set omits:
    @test (Member(:elevator), k, Member(:direction)) in rep.detail.missing_triggers
    # DispatchElevator's direction write is recorded as a near-miss that should
    # have been a trigger (on whichever elevator moved in the short prefix).
    @test any(nm -> nm.class == :container_near_miss &&
                    nm.address[end] == Member(:direction), rep.detail.near_misses)
    block = sprint(show, MIME"text/plain"(), rep)
    @test occursin("direction", block)
    @test occursin("MISSING", block)
    @test count(==('\n'), block) <= 30
end

########## Scenario 2 — OpenElevatorDoors PROPOSED BUT REJECTED (broken guard data) ##########

@testset "whynot: OpenElevatorDoors proposed but rejected (no button)" begin
    seed = 93472934
    rec = RecordSkeleton()
    sim = SimulationFSM(_NB.ElevatorSystem(1, 2, 5), _elev_events(_NB);
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(seed), policy=rec)
    ChronoSim.run(sim, _NB.init_physical, (p, i, e, w) -> w > 60.0)
    skel = recorded_skeleton(rec)

    # The dispatched elevator opened its doors (that OpenElevatorDoors fired); the
    # never-dispatched instance stays rejected for the whole run.
    opened = Set{Int}()
    for st in skel.steps, a in st.changed
        length(a) == 3 && a[1] == Member(:elevator) && a[3] == Member(:doors_open) &&
            push!(opened, a[2])
    end
    @test length(opened) == 1
    k_open = first(opened)
    k_other = 3 - k_open

    factory = policy -> (SimulationFSM(_NB.ElevatorSystem(1, 2, 5), _elev_events(_NB);
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(seed), policy=policy),
        _NB.init_physical)

    rep = whynot(skel, factory, _NB.OpenElevatorDoors(k_other))
    @test rep.stage == :rejected
    @test rep.detail.n_proposals >= 1
    case = rep.detail.examined[1]
    @test case.clause_analysis == :clauses
    @test occursin("call_exists || button_pressed", case.failing_clause)
    # names buttons_pressed or calls[...].requested as the failing read:
    @test any(e -> e.address == (Member(:elevator), k_other, Member(:buttons_pressed)) ||
                   (length(e.address) == 3 && e.address[1] == Member(:calls) &&
                    e.address[3] == Member(:requested)), case.reads)
    # every read's writer is nothing, :init, or a step <= this rejection step.
    @test all(e -> e.writer === nothing || e.writer.clock === :init ||
                   e.writer.step <= case.step, case.reads)
    @test count(==('\n'), sprint(show, MIME"text/plain"(), rep)) <= 30
end

########## Scenario 3 — a stop predicate that can never become true (sirvillage) ##########

@testset "whyrunning: predicate reads state nothing writes" begin
    rng = Xoshiro(2938423)
    physical = SIRVillage.Village(30, 10, 1.0, rng)
    # included_transitions minus Mutate, so no step ever increments next_strain_id.
    included = [SIRVillage.InitEvent, SIRVillage.Travel, SIRVillage.Infect,
        SIRVillage.Recover, SIRVillage.Reset]
    rec = RecordSkeleton()
    sim = SimulationFSM(physical, included; rng=rng, policy=rec)
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, SIRVillage.InitEvent(), (p, i, e, w) -> w > 2.0)
    end
    skel = recorded_skeleton(rec)

    rep = whyrunning(sim, skel, phys -> phys.next_strain_id > 10)
    @test rep.predicate_value == false
    @test any(e -> e.address == (Member(:next_strain_id),), rep.reads)
    @test rep.predicate_writes_in_window == []
    @test rep.reachability == "reachability analysis requires effect analysis (not yet run)"
    block = sprint(show, MIME"text/plain"(), rep)
    @test occursin("next_strain_id", block)
    @test count(==('\n'), block) <= 30
end

########## Scenario 4 — a seeded invariant violation names the guilty writer ##########

# Test-local corruption event: reacts to any person moving, fires early, and sets
# person[1].elevator = 1 while location stays nonzero, breaking the xor invariant.
module WhyVerbsCorrupt
using ChronoSim
using ChronoSim.ObservedState
using Distributions
import ChronoSim: precondition, enable, fire!, generators

struct CorruptPerson <: SimEvent end

@conditionsfor CorruptPerson begin
    @reactto changed(person[who].location) do system
        generate(CorruptPerson())
    end
end

precondition(::CorruptPerson, system) = system.person[1].location != 0
enable(::CorruptPerson, system, when) = (Exponential(0.01), when)
function fire!(::CorruptPerson, system, when, rng)
    system.person[1].elevator = 1
    return nothing
end
end # module

@testset "whystopped: invariant violation names the guilty writer" begin
    physical = ElevatorExample.ElevatorSystem(1, 1, 3)
    transitions = vcat(_elev_events(ElevatorExample), WhyVerbsCorrupt.CorruptPerson)
    rec = RecordSkeleton()
    sim = SimulationFSM(physical, transitions;
        sampler=CombinedNextReaction{Tuple,Float64}(), rng=Xoshiro(93472934),
        policy=PolicyStack(rec, CheckInvariants(ElevatorExample)))
    err = try
        with_logger(ConsoleLogger(stderr, Logging.Warn)) do
            ChronoSim.run(sim, ElevatorExample.init_physical, (p, i, e, w) -> w > 120.0)
        end
        nothing
    catch e
        e
    end
    @test err isa InvariantViolation

    rep = whystopped(err)
    @test rep.kind == :invariant_violation
    @test rep.invariant == "person location xor elevator"
    @test rep.guilty[1].address == (Member(:person), 1, Member(:elevator))
    @test rep.guilty[1].writer.clock == (:CorruptPerson,)
    @test rep.replay_command == err.replay_command
    @test occursin("replay(", sprint(show, MIME"text/plain"(), rep))
    @test count(==('\n'), sprint(show, MIME"text/plain"(), rep)) <= 30
end
