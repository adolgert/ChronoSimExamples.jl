# Differential trajectory equivalence: a hand-written module and its derived-
# generator twin, constructed with identical args + seed and run for identical
# stop conditions, must produce identical trajectories. The framework sorts
# proposed candidates by clock key before enabling (placetoevent.jl), so two
# equivalent generator sets yield the same enabled set and the same trajectory,
# even though the derived generators propose a superset of candidates the
# precondition then filters. A divergence is a real finding, not a tolerance to
# loosen: we surface the first differing (clock_key, when) pair.

using ChronoSim
using CompetingClocks
using Random
using Test

using ChronoSimExamples: ReliabilitySim, ReliabilityDerivedSim
using ChronoSimExamples: ElevatorExample, ElevatorDerivedExample

# Distinct struct types live in distinct modules, so an Enum-typed event field
# (DispatchElevator.direction) carries a module-specific enum value:
# ElevatorExample.Up and ElevatorDerivedExample.Up are unequal despite naming the
# same state. Normalize enum key components to their bare Symbol so the comparison
# tests the trajectory, not the enum's defining module.
_norm_key(k::Tuple) = map(x -> x isa Enum ? Symbol(x) : x, k)
_norm(traj) = [(_norm_key(k), w) for (k, w) in traj]

# Element-wise first divergence between two trajectories, or nothing if equal.
function _first_divergence(a, b)
    n = min(length(a), length(b))
    for i in 1:n
        if a[i] != b[i]
            return (i, a[i], b[i])
        end
    end
    if length(a) != length(b)
        extra = length(a) > length(b) ? (:hand_longer, a[n + 1]) : (:derived_longer, b[n + 1])
        return (n + 1, extra, nothing)
    end
    return nothing
end

function _reliability_trajectory(M; days=10.0, seed=2947223)
    physical = M.IndividualState(15, 10)
    included = [M.StartDay, M.EndDay, M.Break, M.Repair]
    traj = Tuple{Tuple,Float64}[]
    observer = (p, when, event, changed) -> push!(traj, (clock_key(event), when))
    sim = SimulationFSM(
        physical,
        included;
        rng=Xoshiro(seed),
        sampler=CombinedNextReaction{Tuple,Float64}(),
        observer=observer,
    )
    initializer = (init_physical, when, rng) -> M.initialize!(init_physical, rng)
    stop_condition = (p, step_idx, event, when) -> when > days
    ChronoSim.run(sim, initializer, stop_condition)
    return traj
end

function _elevator_trajectory(M; minutes=120.0, seed=93472934)
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
    traj = Tuple{Tuple,Float64}[]
    observer = (p, when, event, changed) -> push!(traj, (clock_key(event), when))
    sim = SimulationFSM(
        physical,
        included;
        rng=Xoshiro(seed),
        sampler=CombinedNextReaction{Tuple,Float64}(),
        observer=observer,
    )
    stop_condition = (p, step_idx, event, when) -> when > minutes
    ChronoSim.run(sim, M.init_physical, stop_condition)
    return traj
end

@testset "reliability hand-written and derived trajectories are identical" begin
    hand = _norm(_reliability_trajectory(ReliabilitySim))
    derived = _norm(_reliability_trajectory(ReliabilityDerivedSim))
    div = _first_divergence(hand, derived)
    if div !== nothing
        i, h, d = div
        @info "reliability trajectory divergence at index $i: hand=$(h) derived=$(d)" length(
            hand
        ) length(derived)
    end
    # The reliability model naturally exhausts its enabled events early (the
    # existing run_reliability also terminates at when ~ 3.9 well before its day
    # limit), so a few-hundred-step run is not feasible here; both twins stop at
    # the same point. The comparison stays meaningful: it exercises StartDay,
    # EndDay, Break, and Repair.
    @test length(hand) >= 10
    @test hand == derived
end

@testset "elevator hand-written and derived trajectories are identical" begin
    hand = _norm(_elevator_trajectory(ElevatorExample))
    derived = _norm(_elevator_trajectory(ElevatorDerivedExample))
    div = _first_divergence(hand, derived)
    if div !== nothing
        i, h, d = div
        @info "elevator trajectory divergence at index $i: hand=$(h) derived=$(d)" length(hand) length(
            derived
        )
    end
    @test length(hand) > 20
    @test hand == derived
end

@testset "derivation_report runs for every converted event type" begin
    for T in (
        ReliabilityDerivedSim.EndDay,
        ReliabilityDerivedSim.Break,
        ReliabilityDerivedSim.Repair,
        ElevatorDerivedExample.PickNewDestination,
        ElevatorDerivedExample.CallElevator,
        ElevatorDerivedExample.OpenElevatorDoors,
        ElevatorDerivedExample.EnterElevator,
        ElevatorDerivedExample.ExitElevator,
    )
        txt = sprint(io -> derivation_report(io, T))
        @test occursin("Derivation report", txt)
    end
end
