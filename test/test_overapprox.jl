# Over-approximation metric: the paper's first real cost data. For each twin pair
# (hand-written vs. `@precondition`-derived), run identical seeds with the candidate
# counters ON and report, per event type, how many candidates each generator set
# PROPOSED and how many the precondition ADMITTED. Derived generators are a superset
# of the hand-written ones, so derived-proposed / hand-proposed is the price of
# derivation. Admissions are equal between twins because the trajectories are
# identical (see test_differential); proposals are not.

using ChronoSim
using CompetingClocks
using CompetingClocks: CombinedNextReaction
using Random
using Test

using ChronoSimExamples: ReliabilitySim, ReliabilityDerivedSim
using ChronoSimExamples: ElevatorExample, ElevatorDerivedExample
using ChronoSimExamples: SIRVillage, SIRVillageDerived
using Logging

# Run a trajectory function with the counters on and return a snapshot of the
# accumulated per-event-type stats. The counters are process-global, so reset
# before and disable after to keep runs independent.
function _run_with_stats(runfn)
    ChronoSim.reset_generation_stats!()
    ChronoSim.collect_generation_stats(true)
    try
        runfn()
    finally
        ChronoSim.collect_generation_stats(false)
    end
    return copy(ChronoSim.generation_stats())
end

function _run_reliability(M; days=10.0, seed=2947223)
    physical = M.IndividualState(15, 10)
    included = [M.StartDay, M.EndDay, M.Break, M.Repair]
    sim = SimulationFSM(
        physical,
        included;
        rng=Xoshiro(seed),
        sampler=CombinedNextReaction{Tuple,Float64}(),
    )
    initializer = (init_physical, when, rng) -> M.initialize!(init_physical, rng)
    stop_condition = (p, step_idx, event, when) -> when > days
    ChronoSim.run(sim, initializer, stop_condition)
    return nothing
end

function _run_elevator(M; minutes=120.0, seed=93472934)
    physical = M.ElevatorSystem(1, 1, 3)
    included = [
        M.PickNewDestination,
        M.CallElevator,
        M.OpenElevatorDoors,
        M.EnterElevator,
        M.ExitElevator,
        M.CloseElevatorDoors,
        M.MoveElevator,
        M.StopElevator,
        M.DispatchElevator,
    ]
    sim = SimulationFSM(
        physical,
        included;
        rng=Xoshiro(seed),
        sampler=CombinedNextReaction{Tuple,Float64}(),
    )
    stop_condition = (p, step_idx, event, when) -> when > minutes
    ChronoSim.run(sim, M.init_physical, stop_condition)
    return nothing
end

# derived-proposed / hand-proposed, with a printable guard for hand-proposed == 0.
function _overapprox_factor(hand_proposed, derived_proposed)
    hand_proposed == 0 && return derived_proposed == 0 ? "1.00" : "inf"
    return string(round(derived_proposed / hand_proposed; digits=2))
end

# Build one readable per-model table and return it as a String for @info. Also
# returns the union of event-type Symbols for the caller's assertions.
_row(a, b, c, d, e) = "  " * rpad(a, 22) * lpad(b, 8) * lpad(c, 8) * lpad(d, 8) * lpad(e, 8)

function _report(model, hand, derived)
    types = sort!(collect(union(keys(hand), keys(derived))))
    io = IOBuffer()
    println(io, "over-approximation report — $(model)")
    println(io, _row("event", "hand_p", "der_p", "admit", "factor"))
    for t in types
        h = get(hand, t, (proposed=0, admitted=0))
        d = get(derived, t, (proposed=0, admitted=0))
        println(
            io,
            _row(
                string(t),
                h.proposed,
                d.proposed,
                d.admitted,
                _overapprox_factor(h.proposed, d.proposed),
            ),
        )
    end
    return String(take!(io)), types
end

# N (actor count) is returned so the Infect over-approximation factor can be read
# against population size: derived proposes O(N) Infect candidates per state change
# where the hand-written generator proposes only co-located pairs.
function _run_sirvillage(M; N=30, L=10, days=15.0, seed=2938423)
    rng = Xoshiro(seed)
    physical = M.Village(N, L, 1.0, rng)
    included = [M.InitEvent, M.Travel, M.Infect, M.Recover, M.Reset, M.Mutate]
    sim = SimulationFSM(physical, included; rng=rng)
    stop_condition = (p, step_idx, event, when) -> when > days
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, M.InitEvent(), stop_condition)
    end
    return nothing
end

@testset "sirvillage over-approximation: derived proposes a superset, equal admissions" begin
    actor_cnt = 30
    hand = _run_with_stats(() -> _run_sirvillage(SIRVillage; N=actor_cnt))
    derived = _run_with_stats(() -> _run_sirvillage(SIRVillageDerived; N=actor_cnt))
    report, types = _report("sirvillage (N=$actor_cnt actors)", hand, derived)
    @info report
    for t in types
        h = get(hand, t, (proposed=0, admitted=0))
        d = get(derived, t, (proposed=0, admitted=0))
        @test h.admitted == d.admitted
        @test d.proposed >= h.proposed
    end
    @test sum(d.admitted for d in values(derived)) > 0
end

@testset "reliability over-approximation: derived proposes a superset, equal admissions" begin
    hand = _run_with_stats(() -> _run_reliability(ReliabilitySim))
    derived = _run_with_stats(() -> _run_reliability(ReliabilityDerivedSim))
    report, types = _report("reliability", hand, derived)
    @info report
    for t in types
        h = get(hand, t, (proposed=0, admitted=0))
        d = get(derived, t, (proposed=0, admitted=0))
        # Same trajectory ⇒ same events newly enabled ⇒ equal admissions.
        @test h.admitted == d.admitted
        # Derived generators are a superset of the hand-written ones.
        @test d.proposed >= h.proposed
    end
    @test sum(d.admitted for d in values(derived)) > 0
end

@testset "elevator over-approximation: derived proposes a superset, equal admissions" begin
    hand = _run_with_stats(() -> _run_elevator(ElevatorExample))
    derived = _run_with_stats(() -> _run_elevator(ElevatorDerivedExample))
    report, types = _report("elevator", hand, derived)
    @info report
    for t in types
        h = get(hand, t, (proposed=0, admitted=0))
        d = get(derived, t, (proposed=0, admitted=0))
        @test h.admitted == d.admitted
        @test d.proposed >= h.proposed
    end
    @test sum(d.admitted for d in values(derived)) > 0
end
