using ChronoSim
using CompetingClocks
using Logging
using Random
using Test

using ChronoSimExamples: SIRVillage

# The model had no smoke test (its "unfinished" state): strains was a fixed-extent
# ObservedVector whose Mutate push! throws FixedExtentError, so any run long enough
# to mutate crashed. With strains as a keyed ObservedDict the run must complete and
# actually fire events (including Mutate, which grows the strain set).
@testset "sirvillage runs to its time horizon and fires events including Mutate" begin
    person_cnt = 30
    location_cnt = 10
    day_length = 1.0
    days = 15.0 * day_length
    rng = Xoshiro(2938423)
    physical = SIRVillage.Village(person_cnt, location_cnt, day_length, rng)
    included = [
        SIRVillage.InitEvent,
        SIRVillage.Travel,
        SIRVillage.Infect,
        SIRVillage.Recover,
        SIRVillage.Reset,
        SIRVillage.Mutate,
    ]
    event_cnt = Ref(0)
    fired = Set{Symbol}()
    observer = function (physical, when, event, changed)
        event_cnt[] += 1
        push!(fired, clock_key(event)[1])
    end
    # Debug policy on in tests: record a skeleton and check every declared
    # invariant after every fired event (RecordSkeleton before CheckInvariants so
    # a violation carries a replayable prefix).
    policy = PolicyStack(RecordSkeleton(), CheckInvariants(SIRVillage))
    sim = SimulationFSM(physical, included; rng=rng, observer=observer, policy=policy)
    stop_condition = (p, step_idx, event, when) -> when > days
    with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ChronoSim.run(sim, SIRVillage.InitEvent(), stop_condition)
    end
    # The stop condition fires when the NEXT event would pass the horizon, so
    # sim.when is the last event just under `days`; completing the run at all is the
    # smoke signal.
    @test sim.when > 0.0
    @test event_cnt[] > 0
    # A born strain (Mutate) is the change that used to crash; new strain ids beyond
    # the founding id 1 confirm the ObservedDict growth path ran.
    @test physical.next_strain_id > 2
end
