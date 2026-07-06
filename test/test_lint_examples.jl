# Phase 3: footprint lints on the four example models.
#
#   * derived twins + landspread lint clean (0 warnings) — derived triggers ARE the
#     read masks, so a derived reader can never miss a trigger (the coverage-oracle
#     agreement on real models);
#   * the hand-written elevator twin's intended trigger narrowings live here as a
#     reviewed 15-entry LintAllow vector (one justification per warning group);
#   * two seeded-bug modules re-introduce the historical StopElevator-direction and
#     OpenElevatorDoors-doors_open trigger omissions and prove `lint` catches them
#     statically, before any run;
#   * static ⊇ dynamic: a smoke run under LintHarvest confirms every runtime
#     enable-dependency edge is covered by a static lint edge.

using ChronoSimExamples
using ChronoSim
using ChronoSim: lint, assert_lint_clean, LintAllow, LintFailure, LintHarvest,
    static_covers_dynamic, warnings, print_lint, PolicyStack, CheckEffects
using Random
using Test

const EE = ChronoSimExamples.ElevatorExample
const ED = ChronoSimExamples.ElevatorDerivedExample
const SV = ChronoSimExamples.SIRVillage
const SVD = ChronoSimExamples.SIRVillageDerived
const RS = ChronoSimExamples.ReliabilitySim
const RSD = ChronoSimExamples.ReliabilityDerivedSim
const LS = ChronoSimExamples.LandSpread

_elev(mod) = [mod.PickNewDestination, mod.CallElevator, mod.OpenElevatorDoors,
    mod.EnterElevator, mod.ExitElevator, mod.CloseElevatorDoors, mod.MoveElevator,
    mod.StopElevator, mod.DispatchElevator]
_sir(mod) = [mod.InitEvent, mod.Travel, mod.Infect, mod.Recover, mod.Reset, mod.Mutate]
_rel(mod) = [mod.StartDay, mod.EndDay, mod.Break, mod.Repair]

# The reviewed allowlist for the hand-written elevator: each entry is an intended
# trigger narrowing where the hand-written @conditionsfor deliberately reacts to a
# narrower address set than the full precondition read set. Safe because the reader
# is (re)proposed through a correlated trigger in every reachable state — the same
# reachability arguments the elevator.jl comments record. The derived twin, which
# reacts to every read, lints with zero warnings; this vector is the mechanical
# inventory of where the hand twin narrows.
const ELEVATOR_ALLOW = LintAllow[
    # waiting flips only alongside a location/destination write that already
    # (re)triggers the reader through its person triggers.
    LintAllow(; reader=:PickNewDestination, mask="[person, ℤ, waiting]",
        reason="waiting is cleared with location; PickNewDestination re-triggers on location"),
    LintAllow(; reader=:CallElevator, mask="[person, ℤ, waiting]",
        reason="waiting toggles with destination; CallElevator triggers on destination"),
    LintAllow(; reader=:CallElevator, mask="[person, ℤ, location]",
        reason="location changes accompany the destination write CallElevator triggers on"),
    LintAllow(; reader=:EnterElevator, mask="[person, ℤ, location]",
        reason="boarding writes location+waiting together; EnterElevator triggers on waiting"),
    LintAllow(; reader=:EnterElevator, mask="[person, ℤ, destination]",
        reason="destination sets accompany waiting; EnterElevator triggers on waiting"),
    # MoveElevator writes floor ALONE, but Move requires !doors_open and Enter
    # requires doors_open — every floor write happens while Enter is already
    # disabled; the enabling doors_open write is a covered trigger.
    LintAllow(; reader=:EnterElevator, mask="[elevator, ℤ, floor]",
        reason="Move requires !doors_open, Enter requires doors_open: floor writes happen while Enter is disabled; the enabling doors_open write is a covered trigger"),
    # Stop's direction write is disabling (Stationary). Dispatch writes direction
    # only on a Stationary elevator, and Stationary ∧ doors_open is unreachable
    # (buttons cannot grow while Stationary; Stop refuses to fire when
    # doors_will_open), so doors are closed at every enabling direction write.
    LintAllow(; reader=:EnterElevator, mask="[elevator, ℤ, direction]",
        reason="Stop's write is disabling; Dispatch writes direction only on a Stationary elevator, and Stationary ∧ doors_open is unreachable, so doors are closed at every enabling direction write"),
    LintAllow(; reader=:ExitElevator, mask="[person, ℤ, elevator]",
        reason="a passenger's elevator field is set with the floor/doors ExitElevator triggers on"),
    LintAllow(; reader=:ExitElevator, mask="[person, ℤ, destination]",
        reason="destination changes cannot enable exit without a floor/doors change triggered"),
    LintAllow(; reader=:CloseElevatorDoors, mask="[person, ℤ, waiting]",
        reason="close re-proposed via fired(Enter/Exit) and doors_open, not person.waiting"),
    LintAllow(; reader=:CloseElevatorDoors, mask="[person, ℤ, destination]",
        reason="close re-proposed via fired(Enter/Exit) and doors_open, not destination"),
    LintAllow(; reader=:CloseElevatorDoors, mask="[elevator, ℤ, floor]",
        reason="floor moves precede a doors_open change that re-triggers close"),
    LintAllow(; reader=:CloseElevatorDoors, mask="[elevator, ℤ, direction]",
        reason="direction sets precede a doors_open change that re-triggers close"),
    LintAllow(; reader=:MoveElevator, mask="[elevator, ℤ, buttons_pressed]",
        reason="button clears accompany a doors_open/floor change MoveElevator triggers on"),
    # Monotone-disable: with direction fixed, a floor change can flip
    # any_approaching only true→false; Dispatch's own floor write comes with a
    # covered direction write.
    LintAllow(; reader=:DispatchElevator, mask="[elevator, ℤ, floor]",
        reason="monotone-disable: with direction fixed, a floor change flips any_approaching only true→false; Dispatch's own floor write comes with a covered direction write"),
]

@testset "lint derived twins clean" begin
    r = lint(_elev(ED); physical=ED.ElevatorSystem(1, 1, 3))
    @test assert_lint_clean(r) === r
    @test isempty(warnings(r))

    rs = lint(_sir(SVD); physical=SVD.Village(30, 10, 1.0, Xoshiro(1)))
    @test assert_lint_clean(rs) === rs

    rr = lint(_rel(RSD); physical=RSD.IndividualState(15, 10))
    @test assert_lint_clean(rr) === rr

    rl = lint([LS.Spread]; physical=LS.Landscape(10, Xoshiro(1)))
    @test assert_lint_clean(rl) === rl
    @test :distance in rl.dead_addresses     # rate-only input, the intended smell
end

@testset "lint hand twins with allowlist" begin
    r = lint(_elev(EE); physical=EE.ElevatorSystem(1, 1, 3))
    @test length(warnings(r)) == 28
    @test_throws LintFailure assert_lint_clean(r)                # without the allowlist
    @test assert_lint_clean(r; allow=ELEVATOR_ALLOW) === r        # with it

    rs = lint(_sir(SV); physical=SV.Village(30, 10, 1.0, Xoshiro(1)))
    @test assert_lint_clean(rs) === rs                           # sirvillage hand: 0 warnings

    rr = lint(_rel(RS); physical=RS.IndividualState(15, 10))
    @test assert_lint_clean(rr) === rr                           # reliability hand: 0 warnings
end

@testset "report text in CI log" begin
    # Greppable full reports in the CI log; the sirvillage report is captured
    # verbatim into docs/src/runbook.md (definition of done 2).
    for (name, r) in [
        ("elevator-hand", lint(_elev(EE); physical=EE.ElevatorSystem(1, 1, 3))),
        ("sirvillage-hand", lint(_sir(SV); physical=SV.Village(30, 10, 1.0, Xoshiro(1)))),
        ("reliability-hand", lint(_rel(RS); physical=RS.IndividualState(15, 10))),
        ("landspread", lint([LS.Spread]; physical=LS.Landscape(10, Xoshiro(1)))),
    ]
        println("----- lint report: $name -----")
        print_lint(stdout, r)
        @test true
    end
end

# ---- Seeded-bug fixtures: the historical StopElevator / doors_open trigger gaps ----
# A compact hand-written elevator flavored module (@fire + @guard) with two
# deliberately-omitted triggers, so lint catches both bugs statically.
module SeededElevatorBugs
using ChronoSim
using ChronoSim.ObservedState
using CompetingClocks
using Distributions
using Random
import ChronoSim: precondition, generators, enable, fire!

@enum Dir Up Down Stationary

@keyedby Elevator Int64 begin
    floor::Int64
    direction::Dir
    doors_open::Bool
    buttons_pressed::Set{Int64}
end
@observedphysical Sys begin
    elevator::ObservedVector{Elevator,Member}
    floor_cnt::Int64
end
function Sys(n::Int)
    e = ObservedArray{Elevator,Member}(undef, n)
    for i in eachindex(e)
        e[i] = Elevator(1, Stationary, false, Set{Int64}())
    end
    return Sys(e, 3)
end

# DispatchElevator writes elevator.direction (and floor) — the StopElevator input.
struct DispatchElevator <: SimEvent
    i::Int64
end
@guard function precondition(evt::DispatchElevator, s)
    return s.elevator[evt.i].direction == Stationary
end
@fire function fire!(evt::DispatchElevator, s, when, rng)
    s.elevator[evt.i].direction = Up
    s.elevator[evt.i].floor = 2
end
@conditionsfor DispatchElevator begin
    @reactto changed(elevator[i].doors_open) do s
        generate(DispatchElevator(i))
    end
end

# StopElevator reads direction/doors_open — BUG: no trigger on elevator.direction.
struct StopElevator <: SimEvent
    i::Int64
end
@guard function precondition(evt::StopElevator, s)
    elevator = s.elevator[evt.i]
    next_floor = elevator.direction == Up ? elevator.floor + 1 : elevator.floor - 1
    next_floor_valid = 1 <= next_floor <= s.floor_cnt
    return !elevator.doors_open && !next_floor_valid
end
@fire function fire!(evt::StopElevator, s, when, rng)
    s.elevator[evt.i].direction = Stationary
end
@conditionsfor StopElevator begin
    @reactto changed(elevator[i].floor) do s
        generate(StopElevator(i))
    end
    @reactto changed(elevator[i].doors_open) do s
        generate(StopElevator(i))
    end
    # BUG: the elevator[i].direction trigger is deliberately omitted.
end

# CloseElevatorDoors writes doors_open — the OpenElevatorDoors input.
struct CloseElevatorDoors <: SimEvent
    i::Int64
end
@guard function precondition(evt::CloseElevatorDoors, s)
    return s.elevator[evt.i].doors_open
end
@fire function fire!(evt::CloseElevatorDoors, s, when, rng)
    s.elevator[evt.i].doors_open = false
end
@conditionsfor CloseElevatorDoors begin
    @reactto changed(elevator[i].doors_open) do s
        generate(CloseElevatorDoors(i))
    end
end

# OpenElevatorDoors reads doors_open/direction/buttons — BUG: no doors_open trigger.
struct OpenElevatorDoors <: SimEvent
    i::Int64
end
@guard function precondition(evt::OpenElevatorDoors, s)
    elevator = s.elevator[evt.i]
    button_pressed = elevator.floor ∈ elevator.buttons_pressed
    return !elevator.doors_open && button_pressed
end
@fire function fire!(evt::OpenElevatorDoors, s, when, rng)
    s.elevator[evt.i].doors_open = true
end
@conditionsfor OpenElevatorDoors begin
    @reactto changed(elevator[i].floor) do s
        generate(OpenElevatorDoors(i))
    end
    @reactto changed(elevator[i].buttons_pressed) do s
        generate(OpenElevatorDoors(i))
    end
    # BUG: the elevator[i].doors_open trigger is deliberately omitted.
end

const EVENTS = [DispatchElevator, StopElevator, CloseElevatorDoors, OpenElevatorDoors]
end # module SeededElevatorBugs

@testset "re-introduced StopElevator bug caught statically" begin
    r = lint(SeededElevatorBugs.EVENTS; physical=SeededElevatorBugs.Sys(1))
    ws = warnings(r)

    stop_dir = [e for e in ws
                if e.reader === :StopElevator &&
                e.overlap_mask == (ChronoSim.Member(:elevator), ChronoSim.MEMBERINDEX, ChronoSim.Member(:direction))]
    @test !isempty(stop_dir)
    @test :DispatchElevator in [e.writer for e in stop_dir]

    open_doors = [e for e in ws
                  if e.reader === :OpenElevatorDoors &&
                  e.overlap_mask == (ChronoSim.Member(:elevator), ChronoSim.MEMBERINDEX, ChronoSim.Member(:doors_open))]
    @test !isempty(open_doors)
    @test :CloseElevatorDoors in [e.writer for e in open_doors]
end

@testset "static covers dynamic (smoke)" begin
    # sirvillage smoke run under the harvest policy (InitEvent is in the vector).
    hsir = LintHarvest()
    SV.run_sirvillage(; policy=PolicyStack(hsir, CheckEffects(_sir(SV))))
    rsir = lint(_sir(SV); physical=SV.Village(30, 10, 1.0, Xoshiro(1)))
    dcs = static_covers_dynamic(rsir, hsir; ignore_writers=[:InitializeEvent])
    @test dcs.covered

    # elevator smoke run (function initializer -> InitializeEvent, ignored).
    hel = LintHarvest()
    EE.run_elevator(; policy=PolicyStack(hel, CheckEffects(_elev(EE))))
    rel = lint(_elev(EE); physical=EE.ElevatorSystem(1, 1, 3))
    dce = static_covers_dynamic(rel, hel; ignore_writers=[:InitializeEvent])
    @test dce.covered
end
