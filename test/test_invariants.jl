using ChronoSim
using ChronoSim: module_invariants, CheckInvariants, InvariantViolation, PolicyStack,
    RecordSkeleton, clock_key, NoPolicy
using CompetingClocks
using Distributions
using Logging
using Random
using Test

using ChronoSimExamples: ElevatorExample, SIRVillage

# A test-local corruption event for the elevator: it reacts to any person moving,
# is enabled fast, and its fire! sets person[1].elevator = 1 while leaving
# location nonzero — breaking "person location xor elevator". Defined outside the
# shipping model so the model stays clean.
module ElevatorCorruptFixture
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
enable(::CorruptPerson, system, when) = (Exponential(0.01), when)   # mean 0.01: fires early
function fire!(::CorruptPerson, system, when, rng)
    system.person[1].elevator = 1   # location stays nonzero -> xor invariant breaks
    return nothing
end
end # module

using .ElevatorCorruptFixture: CorruptPerson

_elevator_invariant(name) =
    only(d for d in module_invariants(ElevatorExample) if d.name == name)
_sirvillage_invariant(name) =
    only(d for d in module_invariants(SIRVillage) if d.name == name)

@testset "old and new elevator checks agree on a violating state" begin
    # Uncorrupted: both the old validator and the new invariant report clean.
    clean = ElevatorExample.ElevatorSystem(2, 1, 3)
    @test isempty(ElevatorExample.validate_type_invariant(clean))
    xor_def = _elevator_invariant("person location xor elevator")
    @test xor_def.checker(clean) == true

    # Corrupted per the plan: person[1] both on a floor and in an elevator.
    corrupt = ElevatorExample.ElevatorSystem(2, 1, 3)
    corrupt.person[1].location = 2
    corrupt.person[1].elevator = 1
    @test !isempty(ElevatorExample.validate_type_invariant(corrupt))
    @test xor_def.checker(corrupt) == false

    # Same pairing for one sirvillage check: individual_cnt vs roster length.
    village = SIRVillage.Village(10, 10, 1.0, Xoshiro(123))
    @test isempty(SIRVillage.validate_invariants(village))
    loc_def = _sirvillage_invariant("location count matches roster")
    @test loc_def.checker(village) == true
    village.locations[1].individual_cnt += 1
    @test !isempty(SIRVillage.validate_invariants(village))
    @test loc_def.checker(village) == false
end

@testset "elevator corruption run yields full payload" begin
    physical = ElevatorExample.ElevatorSystem(1, 1, 3)
    transitions = [
        ElevatorExample.PickNewDestination,
        ElevatorExample.CallElevator,
        ElevatorExample.OpenElevatorDoors,
        ElevatorExample.EnterElevator,
        ElevatorExample.ExitElevator,
        ElevatorExample.CloseElevatorDoors,
        ElevatorExample.MoveElevator,
        ElevatorExample.StopElevator,
        ElevatorExample.DispatchElevator,
        CorruptPerson,
    ]
    rec = RecordSkeleton()
    sim = SimulationFSM(
        physical, transitions;
        sampler=NextReactionMethod(), key_type=Tuple, rng=Xoshiro(93472934),
        policy=PolicyStack(rec, CheckInvariants(ElevatorExample)),
    )
    stop = (p, i, e, w) -> w > 120.0
    e = try
        with_logger(ConsoleLogger(stderr, Logging.Warn)) do
            ChronoSim.run(sim, ElevatorExample.init_physical, stop)
        end
        nothing
    catch err
        err
    end
    @test e isa InvariantViolation
    @test e.name == "person location xor elevator"
    @test e.event == (:CorruptPerson,)
    @test e.guilty == [(Member(:person), 1, Member(:elevator))]
    @test e.skeleton !== nothing
    @test length(e.skeleton.steps) == e.step
    @test e.replay_command == "replay(sim_factory, skeleton; upto=$(e.step - 1))"
end

@testset "elevator smoke runs under CheckInvariants" begin
    dur = with_logger(ConsoleLogger(stderr, Logging.Warn)) do
        ElevatorExample.run_elevator(policy=CheckInvariants(ElevatorExample))
    end
    @test dur > 9.9
end

@testset "sirvillage smoke under checking matches canonical trajectory" begin
    function build_traj(policy)
        rng = Xoshiro(2938423)
        physical = SIRVillage.Village(30, 10, 1.0, rng)
        included = [
            SIRVillage.InitEvent, SIRVillage.Travel, SIRVillage.Infect,
            SIRVillage.Recover, SIRVillage.Reset, SIRVillage.Mutate,
        ]
        traj = Tuple[]
        observer = (p, when, event, changed) -> (push!(traj, (clock_key(event), when)); nothing)
        sim = SimulationFSM(physical, included; rng=rng, observer=observer, policy=policy)
        stop = (p, i, e, w) -> w > 15.0
        with_logger(ConsoleLogger(stderr, Logging.Warn)) do
            ChronoSim.run(sim, SIRVillage.InitEvent(), stop)
        end
        return traj
    end
    base = build_traj(NoPolicy())
    checked = build_traj(PolicyStack(RecordSkeleton(), CheckInvariants(SIRVillage)))
    @test !isempty(base)
    @test checked == base
end
