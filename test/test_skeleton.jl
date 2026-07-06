# The plan's definition-of-done test for Phase 1b: a `RecordSkeleton` policy run
# side by side with a `TrajectorySave` observer on the sirvillage smoke run must
# record exactly the same event sequence the observer sees, and recording must
# not perturb the canonical differential trajectory.

using ChronoSim
using CompetingClocks
using CompetingClocks: CombinedNextReaction
using Random
using Test
using Logging

using ChronoSimExamples: SIRVillage

# One smoke run (same config as test_differential.jl's _sirvillage_trajectory)
# with the TrajectorySave observer and a RecordSkeleton policy installed on the
# same single run. Returns (observer, policy).
function _recorded_sirvillage_run(M; N=30, L=10, days=15.0, seed=2938423)
    rng = Xoshiro(seed)
    physical = M.Village(N, L, 1.0, rng)
    included = [M.InitEvent, M.Travel, M.Infect, M.Recover, M.Reset, M.Mutate]
    ts = M.TrajectorySave()
    rec = RecordSkeleton(; metadata=(model="sirvillage", N=N, seed=seed))
    sim = SimulationFSM(physical, included; rng=rng, observer=ts, policy=rec)
    ts.sim = sim
    stop_condition = (p, step_idx, event, when) -> when > days
    # TrajectorySave logs an @info per event; silence it.
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, M.InitEvent(), stop_condition)
    end
    return ts, rec
end

@testset "sirvillage skeleton equals TrajectorySave record" begin
    ts, rec = _recorded_sirvillage_run(SIRVillage)
    skel = recorded_skeleton(rec)
    # The observer's extra init entry, which the skeleton keeps in `init`, not `steps`.
    @test ts.trajectory[1].event == clock_key(SIRVillage.InitEvent())
    # Exact, tolerance-free equality of the fired event sequence.
    @test [(e.event, e.when) for e in ts.trajectory[2:end]] ==
        [(s.clock, s.when) for s in skel.steps]
    @test length(skel.steps) > 100
end

@testset "sirvillage differential unchanged under recording" begin
    ts, rec = _recorded_sirvillage_run(SIRVillage)
    # The canonical smoke trajectory (observer records InitEvent at index 1).
    # _sirvillage_trajectory is defined in test_differential.jl; this file runs via runtests.jl's include order.
    canonical = _sirvillage_trajectory(SIRVillage)
    @test canonical == [(e.event, e.when) for e in ts.trajectory]
end
